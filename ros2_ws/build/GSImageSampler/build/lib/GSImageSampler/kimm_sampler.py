#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GSImageSampler (no-args): ROS2 → COLMAP 덤프 + CameraInfo 퍼블리셔 (멀티카메라, 자동 config 사용)

- ament index로 <pkg>/share/<pkg>/config 폴더를 자동 검색
- 다음 파일명을 자동 로드: back_calib.json, front_up_calib.json, right_calib.json, front_down_calib.json, left_calib.json
- CameraInfo 퍼블리시(TRANSIENT_LOCAL) + Odometry/Image 트리거 기반 COLMAP 파일(images.txt, cameras.txt)
- 재시작 시 images.txt에서 IMAGE_ID 이어받기
- 카메라별 고정 외부파라미터 T_bc (body->camera) 반영

실행:
  ros2 run GSImageSampler kimm_sampler
"""

import os
import math
import json
import threading
from collections import deque
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from datetime import datetime, timezone, timedelta
KST = timezone(timedelta(hours=9))


# -------------------- 경로/타임스탬프 --------------------

def _make_timestamped_root(base: str = "colmap_dataset") -> str:
    ts = datetime.now(KST).strftime("%Y%m%d_%H%M%S")
    root = f"{base}_{ts}"
    if not os.path.exists(root):
        return root
    idx = 1
    while True:
        cand = f"{root}_{idx:02d}"
        if not os.path.exists(cand):
            return cand
        idx += 1


# -------------------- 선형대수/쿼터니언 유틸 --------------------

def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]
    ], dtype=np.float64)


def R_to_quat(R: np.ndarray) -> np.ndarray:
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return normalize_quat_xyzw(q)


def normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n <= 0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / n


# -------------------- JSON → CameraInfo --------------------

def load_calib_json_to_camerainfo(json_path: str, frame_id: str = "camera") -> CameraInfo:
    """OpenCV-style JSON을 읽어 ROS2 CameraInfo로 변환."""
    with open(json_path, "r") as f:
        data = json.load(f)

    width = int(data["image_size"]["width"])
    height = int(data["image_size"]["height"])
    K = np.array(data["camera_matrix"], dtype=np.float64).reshape(3, 3)
    dist = list(map(float, data.get("dist_coeffs", [])))  # k1,k2,p1,p2,(k3...)

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.k = [K[0,0], K[0,1], K[0,2],
             K[1,0], K[1,1], K[1,2],
             K[2,0], K[2,1], K[2,2]]
    msg.d = dist
    msg.distortion_model = "plumb_bob"  # 표준 radial-tangential
    msg.r = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]
    msg.p = [K[0,0], 0.0,     K[0,2], 0.0,
             0.0,    K[1,1], K[1,2], 0.0,
             0.0,    0.0,    1.0,    0.0]
    return msg


# -------------------- 노드 --------------------

class MultiCamOdometryToColmap(Node):
    def __init__(self):
        super().__init__('multicam_odometry_to_colmap')

        # ===== 패키지/폴더(고정) =====
        package_name = 'GSImageSampler'
        config_dir = 'config'
        try:
            pkg_share = get_package_share_directory(package_name)
        except Exception as e:
            raise RuntimeError(f"[Config] get_package_share_directory('{package_name}') failed: {e}")
        self.config_root = os.path.join(pkg_share, config_dir)

        # ===== 자동 로드 대상 파일명(고정) + 토픽 매핑(고정) =====
        # 필요하면 이 테이블만 수정
        name_topic_pairs = [
            ("back",  "/camera0/image_raw", "/back/camera_info",  "back_calib.json", [-1.5, 0.0, -0.18], [0.0, 0.0, 1.0, 0.0]),
            ("fup",   "/camera/camera/color/image_raw", "/fup/camera_info", "front_up_calib.json", [0.072, 0.032, -0.075], [0.0, 0.0, 0.0, 1.0]),
            ("fdown", "/camera3/image_raw", "/fdown/camera_info", "front_down_calib.json", [0.1, 0.0, -0.18], [0.0, 0.0, 0.0, 1.0]),
            ("left",  "/camera2/image_raw", "/left/camera_info",  "left_calib.json", [-0.701, 0.403, 0.180], [0.0, 0.0, 0.707, 0.707]),
            ("right", "/camera1/image_raw", "/right/camera_info", "right_calib.json", [-0.7, -0.403, -0.18], [0.0, 0.0, -0.707, 0.707]),
        ]
        # ===== 저장 트리거/버퍼 =====
        self.buffer_duration = 1.5          # 각 카메라 이미지 버퍼 유지 [s]
        self.translation_threshold = 0.4     # 저장 트리거: 변위 [m]
        self.rotation_threshold_rad = 0.30   # 저장 트리거: 회전 [rad]
        self.match_tolerance = 0.050         # 타임스탬프 매칭 허용오차 [s]

        # ===== COLMAP 출력 경로 =====
        self.ds_root = _make_timestamped_root("colmap_dataset")
        self.img_dir = os.path.join(self.ds_root, 'images')
        self.ds_join = os.path.join(self.ds_root, 'sparse', 'camera')
        self.images_txt = os.path.join(self.ds_join, 'images.txt')
        self.cameras_txt = os.path.join(self.ds_join, 'cameras.txt')

        # ===== 카메라 구성 생성 =====
        self.cams: List[Dict[str, Any]] = []
        for idx, (name, img_t, info_t, file_name, t_bc_list, q_bc_list) in enumerate(name_topic_pairs):
            self.cams.append(dict(
                name=name,
                camera_id=idx + 1,
                image_topic=img_t,
                caminfo_topic=info_t,
                t_bc=np.array(t_bc_list, dtype=np.float64),
                q_bc=np.array(q_bc_list, dtype=np.float64),
                calib_json=os.path.join(self.config_root, file_name),
                frame_id=name,
            ))


        # ===== 상태 =====
        self.bridge = CvBridge()
        self.img_buffers: Dict[str, deque] = {c['name']: deque() for c in self.cams}  # (t, img)
        self.cam_models: Dict[str, str] = {c['name']: 'PINHOLE' for c in self.cams}   # 또는 'OPENCV'
        self.cam_has_written: Dict[str, bool] = {c['name']: False for c in self.cams}
        self.cam_size_intr: Dict[str, Dict[str, Any]] = {c['name']: {} for c in self.cams}  # width,height,K,dist
        self.last_saved_odom = None
        self.image_id_counter = 1
        self.file_lock = threading.Lock()

        # ===== FS 준비 =====
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.ds_join, exist_ok=True)
        self._prepare_images_txt()
        self._resume_image_counter()

        # ===== QoS =====
        sensor_qos = qos_profile_sensor_data
        odom_qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ===== CameraInfo 퍼블리셔 (latched) =====
        self.caminfo_pubs: Dict[str, Any] = {}
        self.caminfo_msgs: Dict[str, CameraInfo] = {}
        caminfo_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,   # latched
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for c in self.cams:
            pub = self.create_publisher(CameraInfo, c['caminfo_topic'], caminfo_qos)
            self.caminfo_pubs[c['name']] = pub

            if os.path.isfile(c['calib_json']):
                try:
                    msg = load_calib_json_to_camerainfo(c['calib_json'], frame_id=c.get('frame_id', c['name']))
                    self.caminfo_msgs[c['name']] = msg
                    # 내부 intrinsics 캐시 → cameras.txt 보장
                    K = np.array(msg.k, dtype=np.float64).reshape(3,3)
                    dist = np.array(msg.d, dtype=np.float64)
                    self.cam_size_intr[c['name']] = dict(K=K, dist=dist, width=int(msg.width), height=int(msg.height))
                    self.cam_models[c['name']] = 'OPENCV' if (len(msg.d) >= 4) else 'PINHOLE'
                    msg.header.stamp = self.get_clock().now().to_msg()
                    pub.publish(msg)
                    self.get_logger().info(f"[CameraInfo] Published from JSON for {c['name']}: {c['calib_json']}")
                except Exception as e:
                    self.get_logger().error(f"[CameraInfo] Failed to load {c['calib_json']}: {e}")
            else:
                self.get_logger().warning(f"[CameraInfo] Missing file: {c['calib_json']} (waiting for subscribed CameraInfo)")

        # subscriber late-join 대비 주기 재퍼블리시
        self.create_timer(1.0, self._republish_caminfo_if_any)

        # ===== 구독 =====
        self.odom_sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.odom_cb, odom_qos_best_effort)

        for c in self.cams:
            self.create_subscription(Image, c['image_topic'],
                                     self._make_image_cb(c['name']), sensor_qos)
            self.create_subscription(CameraInfo, c['caminfo_topic'],
                                     self._make_caminfo_cb(c['name']), sensor_qos)

        # JSON 존재성 사전 검증
        for c in self.cams:
            if not os.path.isfile(c['calib_json']):
                self.get_logger().warning(f"[Config] Not found: {c['calib_json']}")

        self.get_logger().info(f"Started: Multi-Cam (N={len(self.cams)}) → COLMAP dumper + CameraInfo publisher")
        self.get_logger().info(f"Config root: {self.config_root}")
        self.get_logger().info(f"Saving to: {os.path.abspath(self.ds_root)}")

    # -------------------- 파일 시스템 유틸 --------------------

    def _prepare_images_txt(self):
        if not os.path.exists(self.images_txt):
            with open(self.images_txt, 'w') as f:
                f.write("# Image list with two lines of data per image:\n")
                f.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n")
                f.write("# POINTS2D[] as (X Y POINT3D_ID)\n")
            self.get_logger().info(f'[COLMAP] Created {self.images_txt}')
        else:
            self.get_logger().info(f'[COLMAP] Using existing {self.images_txt}')

        if not os.path.exists(self.cameras_txt):
            with open(self.cameras_txt, 'w') as f:
                f.write("# Camera list with one line of data per camera:\n")
                f.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
            self.get_logger().info(f'[COLMAP] Created {self.cameras_txt}')
        else:
            self.get_logger().info(f'[COLMAP] Using existing {self.cameras_txt}')

    def _resume_image_counter(self):
        last_id = 0
        try:
            with open(self.images_txt, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if len(parts) >= 10:
                        try:
                            last_id = max(last_id, int(parts[0]))
                        except Exception:
                            pass
        except FileNotFoundError:
            pass
        self.image_id_counter = last_id + 1
        if self.image_id_counter > 1:
            self.get_logger().info(f'[Resume] Next IMAGE_ID = {self.image_id_counter}')

    def _append_camera_line_once(self, cam_name: str, camera_id: int,
                                 width: int, height: int,
                                 model: str,
                                 fx: float, fy: float, cx: float, cy: float,
                                 dist: Optional[np.ndarray]):
        """카메라별 1회만 cameras.txt에 라인 추가 (중복 방지).
        OPENCV 모델에는 k1,k2,p1,p2 (4개)만 기록. (k3는 무시)
        """
        if self.cam_has_written[cam_name]:
            return

        exists = False
        try:
            with open(self.cameras_txt, 'r') as f:
                for line in f:
                    if line.strip().startswith('#') or not line.strip():
                        continue
                    parts = line.split()
                    if len(parts) >= 1:
                        try:
                            if int(parts[0]) == camera_id:
                                exists = True
                                break
                        except Exception:
                            pass
        except FileNotFoundError:
            pass

        with self.file_lock:
            with open(self.cameras_txt, 'a') as f:
                if not exists:
                    if model.upper() == 'OPENCV' and dist is not None and len(dist) >= 4:
                        k1, k2, p1, p2 = float(dist[0]), float(dist[1]), float(dist[2]), float(dist[3])
                        f.write(f"{camera_id} OPENCV {width} {height} "
                                f"{fx} {fy} {cx} {cy} {k1} {k2} {p1} {p2}\n")
                    else:
                        f.write(f"{camera_id} PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n")
                    f.flush()
                    os.fsync(f.fileno())

        self.cam_has_written[cam_name] = True
        self.get_logger().info(f'[COLMAP] cameras.txt appended: {cam_name} (ID={camera_id}, model={model})')

    # -------------------- CameraInfo 퍼블리셔 보조 --------------------

    def _republish_caminfo_if_any(self):
        now = self.get_clock().now().to_msg()
        for name, msg in self.caminfo_msgs.items():
            msg.header.stamp = now
            self.caminfo_pubs[name].publish(msg)

    # -------------------- 콜백 바인더 --------------------

    def _make_caminfo_cb(self, cam_name: str):
        def caminfo_cb(msg: CameraInfo):
            try:
                K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                dist = np.array(msg.d, dtype=np.float64) if msg.d else None
                width = int(msg.width)
                height = int(msg.height)

                self.cam_size_intr[cam_name] = dict(K=K, dist=dist, width=width, height=height)
                model = 'OPENCV' if (dist is not None and len(dist) >= 4) else 'PINHOLE'
                self.cam_models[cam_name] = model

                camera_id = next(c['camera_id'] for c in self.cams if c['name'] == cam_name)
                fx = float(K[0, 0]) if K is not None else 500.0
                fy = float(K[1, 1]) if K is not None else 500.0
                cx = float(K[0, 2]) if K is not None else (width * 0.5)
                cy = float(K[1, 2]) if K is not None else (height * 0.5)

                self._append_camera_line_once(cam_name, camera_id, width, height, model, fx, fy, cx, cy, dist)
            except Exception as e:
                self.get_logger().warning(f'[{cam_name}][CameraInfo] parse failed: {e}')
        return caminfo_cb

    def _make_image_cb(self, cam_name: str):
        def image_cb(msg: Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                buf = self.img_buffers[cam_name]
                buf.append((t, cv_image))
                while buf and (t - buf[0][0]) > self.buffer_duration:
                    buf.popleft()

                if not self.cam_has_written[cam_name]:
                    sz = self.cam_size_intr.get(cam_name, {})
                    width = int(sz.get('width', cv_image.shape[1]))
                    height = int(sz.get('height', cv_image.shape[0]))
                    K = sz.get('K', None)
                    dist = sz.get('dist', None)
                    model = self.cam_models.get(cam_name, 'PINHOLE')
                    if K is None:
                        fx = fy = 500.0
                        cx = width * 0.5
                        cy = height * 0.5
                    else:
                        fx = float(K[0, 0]); fy = float(K[1, 1]); cx = float(K[0, 2]); cy = float(K[1, 2])
                    camera_id = next(c['camera_id'] for c in self.cams if c['name'] == cam_name)
                    self._append_camera_line_once(cam_name, camera_id, width, height, model, fx, fy, cx, cy, dist)

            except Exception as e:
                self.get_logger().error(f'[{cam_name}][Image] Conversion error: {e}')
        return image_cb

    # -------------------- 오도메트리 콜백 --------------------

    def odom_cb(self, msg: Odometry):
        try:
            # 프로젝트 좌표계 맞춤 예시(필요 시 수정)
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            z = float(msg.pose.pose.position.z)
            qx = float(msg.pose.pose.orientation.x)
            qy = float(msg.pose.pose.orientation.y)
            qz = float(msg.pose.pose.orientation.z)
            qw = float(msg.pose.pose.orientation.w)

            q_xyzw = normalize_quat_xyzw(np.array([qx, qy, qz, qw], dtype=np.float64))
            R_wb = quat_to_R(q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3])
            t_wb = np.array([x, y, z], dtype=np.float64)

            if self.last_saved_odom is None:
                self.last_saved_odom = {'t_wb': t_wb, 'R_wb': R_wb}
                return

            d_t = np.linalg.norm(t_wb - self.last_saved_odom['t_wb'])
            R_prev = self.last_saved_odom['R_wb']
            R_rel = R_prev.T @ R_wb
            c = max(-1.0, min(1.0, (np.trace(R_rel) - 1.0) * 0.5))
            angle = math.acos(c)

            if (d_t > self.translation_threshold) or (angle > self.rotation_threshold_rad):
                odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.get_logger().info(f"[Trigger] Δtrans={d_t:.3f} m, Δrot={angle:.3f} rad @ t={odom_time:.6f}")
                any_saved = self._save_for_all_cams(odom_time, R_wb, t_wb)
                if any_saved:
                    self.last_saved_odom = {'t_wb': t_wb, 'R_wb': R_wb}

        except Exception as e:
            self.get_logger().error(f'[Odom] Callback error: {e}')

    # -------------------- 저장 --------------------

    def _find_best_image(self, buf: deque, target_t: float, tol: float) -> Optional[Tuple[float, np.ndarray]]:
        if not buf:
            return None
        best = None
        best_abs = float('inf')
        for t, img in buf:
            diff = abs(t - target_t)
            if diff < best_abs:
                best_abs = diff
                best = (t, img)
        if best is None or best_abs > tol:
            return None
        return best
    def _save_for_all_cams(self, odom_time: float, R_wb: np.ndarray, t_wb: np.ndarray) -> bool:
        """
        모든 카메라에 대해, odom_time에 가장 가까운 이미지를 찾아 저장하고
        COLMAP의 images.txt에 (world->camera) 포즈를 기록한다.
        내부 상태(R_wb, t_wb, T_bc)는 ENU(앞 x, 좌 y, 위 z) 기준으로 유지하고,
        파일에 쓸 때 Optical(우 x, 아래 y, 앞 z; REP-103)로 변환한다.
        """
        # ENU -> Optical 고정 회전
        R_OC = np.array([
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
            [1.0,  0.0,  0.0]
        ], dtype=np.float64)
    
        saved_any = False
        for c in self.cams:
            name = c['name']
            camera_id = c['camera_id']
            t_bc = c['t_bc']
            q_bc = normalize_quat_xyzw(c['q_bc'])
            buf = self.img_buffers[name]
    
            # 1) 타임스탬프 매칭
            match = self._find_best_image(buf, odom_time, self.match_tolerance)
            if match is None:
                if buf:
                    t0 = buf[0][0]; t1 = buf[-1][0]; span = t1 - t0
                    self.get_logger().warning(
                        f"[{name}][MatchMiss] No image within ±{self.match_tolerance*1000:.0f} ms @ t={odom_time:.6f}. "
                        f"Buffer=[{t0:.6f},{t1:.6f}] span={span:.3f}s")
                else:
                    self.get_logger().warning(f"[{name}][MatchMiss] Image buffer empty at trigger time.")
                continue

            t_img, img = match
            skew = t_img - odom_time
            if abs(skew) > 0:
                self.get_logger().info(f"[{name}][Match] use t={t_img:.6f} (skew {skew*1000:.1f} ms)")
    
            # 2) body->camera 회전
            R_bc = quat_to_R(q_bc[0], q_bc[1], q_bc[2], q_bc[3])
    
            # 3) 카메라 중심(world)과 world->camera 포즈 (ENU 기준)
            C_w = (R_wb @ t_bc + t_wb)
            R_cw = (R_wb @ R_bc).T
            t_cw = - R_cw @ C_w
    
            # 4) Optical 프레임으로 변환
            R_cw_opt = R_OC @ R_cw
            t_cw_opt = R_OC @ t_cw
    
            # 5) 쿼터니언 재계산 (행렬 -> [x,y,z,w])
            q_cw_xyzw_opt = R_to_quat(R_cw_opt)
            qw = float(q_cw_xyzw_opt[3])
            qx = float(q_cw_xyzw_opt[0])
            qy = float(q_cw_xyzw_opt[1])
            qz = float(q_cw_xyzw_opt[2])
    
            # 6) 파일명 생성(충돌 회피)
            img_name = f'{name}_image_{self.image_id_counter:06d}.png'
            img_path = os.path.join(self.img_dir, img_name)
            while os.path.exists(img_path):
                self.image_id_counter += 1
                img_name = f'{name}_image_{self.image_id_counter:06d}.png'
                img_path = os.path.join(self.img_dir, img_name)
    
            # 7) 이미지 저장
            try:
                ok = cv2.imwrite(img_path, img)
                if not ok:
                    self.get_logger().error(f'[{name}][FS] OpenCV failed to write: {img_path}')
                    continue
            except Exception as e:
                self.get_logger().error(f'[{name}][FS] Exception writing image {img_path}: {e}')
                continue
    
            # 8) images.txt append (COLMAP format, Optical 기준의 world->camera)
            try:
                line1 = (f"{self.image_id_counter} "
                         f"{qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g} "
                         f"{t_cw_opt[0]:.17g} {t_cw_opt[1]:.17g} {t_cw_opt[2]:.17g} "
                         f"{camera_id} {img_name}\n")
                line2 = "\n"
    
                with self.file_lock:
                    with open(self.images_txt, 'a') as f:
                        f.write(line1); f.write(line2)    
                        f.flush(); os.fsync(f.fileno())    
    
                self.get_logger().info(f"[COLMAP] Saved IMAGE_ID={self.image_id_counter} ({name}) -> {img_name}")
                self.image_id_counter += 1
                saved_any = True
    
            except Exception as e:
                self.get_logger().error(f'[{name}][FS] Failed to append to {self.images_txt}: {e}')
                try:
                    if os.path.exists(img_path):
                        os.remove(img_path)
                        self.get_logger().warning(f'[{name}][Rollback] Removed {img_path} due to images.txt failure.')
                except Exception as e2:
                    self.get_logger().error(f'[{name}][Rollback] Failed to remove {img_path}: {e2}')
                continue
    
        return saved_any


# -------------------- main --------------------

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamOdometryToColmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'[Main] Spin error: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f'[Shutdown] destroy_node failed: {e}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()

