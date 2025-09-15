#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 → COLMAP 데이터셋 덤프 노드 (이미지 + images.txt + cameras.txt)

- Odometry와 Image를 받아서, 일정 변위/회전량 이상일 때 이미지 저장 및 COLMAP 포즈(world->camera) 기록
- CameraInfo가 들어오면 1회 cameras.txt 자동 생성(PINHOLE 또는 OPENCV)
- 재시작 시 images.txt에서 IMAGE_ID 이어받기
- T_bc (body->camera) 고정 외부파라미터 반영

Author: you
"""

import os
import math
import threading
from collections import deque
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# -------------------- 선형대수/쿼터니언 유틸 --------------------

def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion(x,y,z,w) -> Rotation matrix (right-handed, passive)."""
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=np.float64)


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> Quaternion(x,y,z,w), numerically stable."""
    m00, m01, m02 = R[0,0], R[0,1], R[0,2]
    m10, m11, m12 = R[1,0], R[1,1], R[1,2]
    m20, m21, m22 = R[2,0], R[2,1], R[2,2]
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


# -------------------- 노드 --------------------

class OdometryImageToColmap(Node):
    def __init__(self):
        super().__init__('odometry_image_to_colmap')

        # ====== 설정 (필요 시 파라미터로 바꾸세요) ======
        self.buffer_duration = 3.0                 # 이미지 버퍼 유지 시간 [s]
        self.translation_threshold = 0.4          # 저장 트리거: 변위 [m]
        self.rotation_threshold_rad = 0.30         # 저장 트리거: 회전 [rad]
        self.match_tolerance = 0.050               # 타임스탬프 매칭 허용오차 [s]
        # body->camera 고정 외부파라미터 (m, quaternion x,y,z,w)
        self.t_bc = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.q_bc = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        # 경로
        self.ds_root = 'colmap_dataset'
        self.img_dir = os.path.join(self.ds_root, 'images')
        self.ds_join = os.path.join(self.ds_root, 'sparse', 'camera')
        self.images_txt = os.path.join(self.ds_join, 'images.txt')
        self.cameras_txt = os.path.join(self.ds_join, 'cameras.txt')
        self.camera_model = 'pinhole'  # 'PINHOLE' 또는 'OPENCV'
        self.camera_id = 1

        # 상태
        self.bridge = CvBridge()
        self.image_buffer: deque[Tuple[float, np.ndarray]] = deque()
        self.last_saved_odom = None  # {'t_wb':np(3,), 'R_wb':np(3,3)}
        self.image_id_counter = 1
        self.has_written_cameras = False
        self.file_lock = threading.Lock()

        # FS 준비
        self.get_logger().info(f"saving path : {os.path.abspath(self.ds_root)}")

        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.ds_join, exist_ok=True)
        self._prepare_images_txt()
        self._resume_image_counter()


        sensor_qos = qos_profile_sensor_data
        odom_qos_best_effort = QoSProfile(
	    reliability=ReliabilityPolicy.BEST_EFFORT,
	    durability=DurabilityPolicy.VOLATILE,
	    history=QoSHistoryPolicy.KEEP_LAST,
	    depth=10,
	)
        self.odom_sub = self.create_subscription(
            Odometry, '/lio_sam/mapping/odometry', self.odom_cb, odom_qos_best_effort)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, sensor_qos)
        # 선택: 카메라 파라미터 자동 기록
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.caminfo_cb, sensor_qos)

        self.get_logger().info('Started: Odometry + Image → COLMAP dumper')

    # ------------- 파일 시스템 유틸 -------------

    def _prepare_images_txt(self):
        if not os.path.exists(self.images_txt):
            with open(self.images_txt, 'w') as f:
                f.write("# Image list with two lines of data per image:\n")
                f.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n")
                f.write("# POINTS2D[] as (X Y POINT3D_ID)\n")
            self.get_logger().info(f'[COLMAP] Created {self.images_txt}')
        else:
            self.get_logger().info(f'[COLMAP] Using existing {self.images_txt}')

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

    def _write_cameras_txt_once(self, width: int, height: int,
                                K: Optional[np.ndarray] = None,
                                dist: Optional[np.ndarray] = None):
        """
        CameraInfo가 들어오면 1회 cameras.txt 생성.
        - PINHOLE: fx fy cx cy
        - OPENCV : fx fy cx cy k1 k2 p1 p2 (rad-tan 4계수만 사용)
        """
        if self.has_written_cameras:
            return
        if os.path.exists(self.cameras_txt):
            self.get_logger().info(f'[COLMAP] Using existing {self.cameras_txt}')
            self.has_written_cameras = True
            return

        # 기본값 (카메라 정보가 없다면 적당한 예시값)
        fx = fy = 500.0
        cx = width * 0.5
        cy = height * 0.5
        k1 = k2 = p1 = p2 = 0.0

        if K is not None:
            fx = float(K[0, 0])
            fy = float(K[1, 1])
            cx = float(K[0, 2])
            cy = float(K[1, 2])

        if dist is not None and len(dist) >= 4:
            k1, k2, p1, p2 = [float(dist[i]) for i in range(4)]
            self.camera_model = 'OPENCV'

        with open(self.cameras_txt, 'w') as f:
            f.write("# Camera list with one line of data per camera:\n")
            f.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
            if self.camera_model == 'OPENCV':
                f.write(f"{self.camera_id} OPENCV {width} {height} "
                        f"{fx} {fy} {cx} {cy} {k1} {k2} {p1} {p2}\n")
            else:
                f.write(f"{self.camera_id} PINHOLE {width} {height} {fx} {fy} {cx} {cy}\n")

        self.has_written_cameras = True
        self.get_logger().info(f'[COLMAP] Created {self.cameras_txt} (model={self.camera_model})')

    # ------------- ROS 콜백 -------------

    def caminfo_cb(self, msg: CameraInfo):
        try:
            # msg.k: row-major 3x3
            K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            dist = np.array(msg.d, dtype=np.float64) if msg.d else None
            width = int(msg.width)
            height = int(msg.height)
            self._write_cameras_txt_once(width, height, K, dist)
        except Exception as e:
            self.get_logger().warning(f'[CameraInfo] parse failed: {e}')

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.image_buffer.append((t, cv_image))

            # 최초 1회 카메라 파일 생성(카메라정보가 오지 않는 환경 대비)
            if not self.has_written_cameras:
                self._write_cameras_txt_once(
                    width=cv_image.shape[1], height=cv_image.shape[0],
                    K=None, dist=None
                )

            # 오래된 프레임 정리
            while self.image_buffer and (t - self.image_buffer[0][0]) > self.buffer_duration:
                self.image_buffer.popleft()

        except Exception as e:
            self.get_logger().error(f'[Image] Conversion error: {e}')

    def odom_cb(self, msg: Odometry):
        try:
            # body in world
            x = -1 * float(msg.pose.pose.position.y)
            y = -1 * float(msg.pose.pose.position.z) 
            z = float(msg.pose.pose.position.x)
            qx = -1 * float(msg.pose.pose.orientation.y)
            qy = -1 * float(msg.pose.pose.orientation.z)
            qz = float(msg.pose.pose.orientation.x)
            qw = float(msg.pose.pose.orientation.w)
            self.get_logger().info(f"It recieves Odom Topic!")

            q_xyzw = normalize_quat_xyzw(np.array([qx, qy, qz, qw], dtype=np.float64))
            R_wb = quat_to_R(q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3])
            t_wb = np.array([x, y, z], dtype=np.float64)

            if self.last_saved_odom is None:
                self.last_saved_odom = {'t_wb': t_wb, 'R_wb': R_wb}
                return

            d_t = np.linalg.norm(t_wb - self.last_saved_odom['t_wb'])

            R_prev = self.last_saved_odom['R_wb']
            R_rel = R_prev.T @ R_wb
            # 수치 클램프
            c = max(-1.0, min(1.0, (np.trace(R_rel) - 1.0) * 0.5))
            angle = math.acos(c)

            if (d_t > self.translation_threshold) or (angle > self.rotation_threshold_rad):
                odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.get_logger().info(f"[Trigger] Δtrans={d_t:.3f} m, Δrot={angle:.3f} rad @ t={odom_time:.6f}")
                if self._save_one_at_time(odom_time, R_wb, t_wb):
                    # 성공적으로 저장했을 때만 기준 갱신
                    self.last_saved_odom = {'t_wb': t_wb, 'R_wb': R_wb}

        except Exception as e:
            self.get_logger().error(f'[Odom] Callback error: {e}')

    # ------------- 코어 로직 -------------

    def _find_best_image(self, target_t: float, tol: float) -> Optional[Tuple[float, np.ndarray]]:
        if not self.image_buffer:
            return None
        best = None
        best_abs = float('inf')
        for t, img in self.image_buffer:
            diff = abs(t - target_t)
            if diff < best_abs:
                best_abs = diff
                best = (t, img)
        if best is None or best_abs > tol:
            return None
        return best

    def _save_one_at_time(self, odom_time: float, R_wb: np.ndarray, t_wb: np.ndarray) -> bool:
        match = self._find_best_image(odom_time, self.match_tolerance)
        if match is None:
            if self.image_buffer:
                t0 = self.image_buffer[0][0]
                t1 = self.image_buffer[-1][0]
                span = t1 - t0
                self.get_logger().warning(
                    f"[MatchMiss] No image within ±{self.match_tolerance*1000:.0f} ms at t={odom_time:.6f}. "
                    f"Buffer=[{t0:.6f}, {t1:.6f}] span={span:.3f}s")
            else:
                self.get_logger().warning("[MatchMiss] Image buffer empty at trigger time.")
            return False

        t_img, img = match
        skew = t_img - odom_time
        if abs(skew) > 0:
            self.get_logger().info(f"[Match] Use image t={t_img:.6f} (skew {skew*1000:.1f} ms)")

        q_bc_xyzw = normalize_quat_xyzw(self.q_bc)

        # q_bc가 body->camera_link(ROS)일 때: optical로 변환
        R_bc = quat_to_R(q_bc_xyzw[0], q_bc_xyzw[1], q_bc_xyzw[2], q_bc_xyzw[3])
        
        
        def rotx(rad: float) -> np.ndarray:
            c, s = math.cos(rad), math.sin(rad)
            return np.array([[1, 0, 0],
                             [0, c, -s],
                             [0, s,  c]], dtype=np.float64)

        def roty(rad: float) -> np.ndarray:
            c, s = math.cos(rad), math.sin(rad)
            return np.array([[ c, 0, s],
                             [ 0, 1, 0],
                             [-s, 0, c]], dtype=np.float64)

        def rotz(rad: float) -> np.ndarray:
            c, s = math.cos(rad), math.sin(rad)
            return np.array([[ c, -s, 0],
                             [ s,  c, 0],
                             [ 0,  0, 1]], dtype=np.float64)
        #R_bc = R_bc @ rotz(math.pi/2)

        t_bc = self.t_bc  # body 좌표계에서의 카메라 중심 위치이므로 그대로 사용

        # camera center in world
        C_w = (R_wb @ t_bc + t_wb)
        #C_w = (R_wb @ t_bc + t_wb)
        R_cw = (R_wb @ R_bc).T
        #R_cw = ( R_wb @ R_bc).T
        # t_cw = -R_cw * C_w
        t_cw = - R_cw @ C_w
       
        q_cw_xyzw = R_to_quat(R_cw)  # normalized
        qw, qx, qy, qz = float(q_cw_xyzw[3]), float(q_cw_xyzw[0]), float(q_cw_xyzw[1]), float(q_cw_xyzw[2])

        # --- 이미지 파일명/경로 (중복 방지) ---
        img_name = f'image_{self.image_id_counter:06d}.png'
        img_path = os.path.join(self.img_dir, img_name)
        while os.path.exists(img_path):  # 매우 드물지만 보호
            self.image_id_counter += 1
            img_name = f'image_{self.image_id_counter:06d}.png'
            img_path = os.path.join(self.img_dir, img_name)

        # --- 파일 쓰기 (락 + fsync) ---
        try:
            ok = cv2.imwrite(img_path, img)
            if not ok:
                self.get_logger().error(f'[FS] OpenCV failed to write: {img_path}')
                return False
        except Exception as e:
            self.get_logger().error(f'[FS] Exception writing image {img_path}: {e}')
            return False

        try:
            line1 = (f"{self.image_id_counter} "
                     f"{qw:.17g} {qx:.17g} {qy:.17g} {qz:.17g} "
                     f"{t_cw[0]:.17g} {t_cw[1]:.17g} {t_cw[2]:.17g} "
                     f"{self.camera_id} {img_name}\n")
            line2 = "\n"  # empty 2D points line

            with self.file_lock:
                with open(self.images_txt, 'a') as f:
                    f.write(line1)
                    f.write(line2)
                    f.flush()
                    os.fsync(f.fileno())

            self.get_logger().info(f"[COLMAP] Saved IMAGE_ID={self.image_id_counter} -> {img_name}")
            self.image_id_counter += 1
            return True

        except Exception as e:
            self.get_logger().error(f'[FS] Failed to append to {self.images_txt}: {e}')
            # 롤백(이미지 제거)
            try:
                if os.path.exists(img_path):
                    os.remove(img_path)
                    self.get_logger().warning(f'[Rollback] Removed {img_path} due to images.txt failure.')
            except Exception as e2:
                self.get_logger().error(f'[Rollback] Failed to remove {img_path}: {e2}')
            return False


# -------------------- main --------------------

def main(args=None):
    rclpy.init(args=args)
    node = OdometryImageToColmap()
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

