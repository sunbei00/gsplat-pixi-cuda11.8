#!/usr/bin/env python3
import os
import glob
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from cv_bridge import CvBridge
import cv2

# ===== 사용자 매핑: 토픽 <-> 이미지 폴더 =====
TOPIC_DIR_MAP = {
    "/camera0/image_raw": "/root/nas/Orchard_DataSet/camera_calibration/back/images",
    "/camera/camera/color/image_raw": "/root/nas/Orchard_DataSet/camera_calibration/front_up/images",
    "/camera3/image_raw": "/root/nas/Orchard_DataSet/camera_calibration/front_down/images",
    "/camera2/image_raw": "/root/nas/Orchard_DataSet/camera_calibration/left/images",
    "/camera1/image_raw": "/root/nas/Orchard_DataSet/camera_calibration/right/images",
}

ODOM_TOPIC = "/lio_sam/mapping/odometry"
PUBLISH_PERIOD = 0.1  # seconds (10Hz)
X_STEP_M = 0.01        # 10 cm per tick

class MultiImageAndOdomPub(Node):
    def __init__(self):
        super().__init__("multi_image_and_odom_pub")

        # Publishers for images
        self.bridge = CvBridge()
        self.image_pubs = {}
        self.image_lists = {}
        self.image_indices = {}
        for topic, d in TOPIC_DIR_MAP.items():
            files = sorted(glob.glob(os.path.join(d, "*.png")))
            if not files:
                self.get_logger().warn(f"[{topic}] 폴더에 PNG가 없습니다: {d}")
            self.image_pubs[topic] = self.create_publisher(Image, topic, 10)
            self.image_lists[topic] = files
            self.image_indices[topic] = 0

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)

        # State for synthetic time & pose
        self.sim_time_sec = 0.0
        self.x = 0.0

        # Timer
        self.timer = self.create_timer(PUBLISH_PERIOD, self.on_timer)

        # For finish detection
        self.total_images = {t: len(self.image_lists[t]) for t in self.image_lists}
        self.get_logger().info("퍼블리셔 준비 완료. 0.1초마다 이미지 + 오도메트리 송신 시작.")

    def on_timer(self):
        # 1) Publish images (one frame per topic per tick, if available)
        all_done = True
        for topic, pub in self.image_pubs.items():
            idx = self.image_indices[topic]
            files = self.image_lists[topic]
            if idx < len(files):
                all_done = False
                img_path = files[idx]
                cv_img = cv2.imread(img_path, cv2.IMREAD_COLOR)
                if cv_img is None:
                    self.get_logger().warn(f"[{topic}] 이미지를 읽지 못했습니다: {img_path}")
                else:
                    msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    # 수동 timestamp (0.0, 0.1, 0.2, …)
                    msg.header.stamp.sec = int(self.sim_time_sec)
                    msg.header.stamp.nanosec = int((self.sim_time_sec % 1.0) * 1e9)
                    # 프레임 아이디는 간단히 topic명에서 추정
                    msg.header.frame_id = topic.replace("/", "_")[1:]  # 예: camera0_image_raw
                    pub.publish(msg)
                    self.get_logger().debug(f"IMG {topic} -> {os.path.basename(img_path)} @ t={self.sim_time_sec:.1f}s")
                self.image_indices[topic] += 1
            else:
                # 해당 토픽은 소진됨
                continue

        # 2) Publish odometry (temp pose)
        odom = Odometry()
        odom.header.frame_id = "odom"       # 월드 좌표계
        odom.child_frame_id = "base_link"   # 로봇 바디
        odom.header.stamp.sec = int(self.sim_time_sec)
        odom.header.stamp.nanosec = int((self.sim_time_sec % 1.0) * 1e9)
        odom.pose.pose.position = Point(x=self.x, y=0.0, z=0.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # 속도: 0.1m / 0.1s = 1.0 m/s
        odom.twist.twist.linear = Vector3(x=1.0, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.odom_pub.publish(odom)
        self.get_logger().info(f"ODOM x={self.x:.2f} t={self.sim_time_sec:.1f}s")

        # 3) Advance synthetic time and pose
        self.sim_time_sec += PUBLISH_PERIOD
        self.x += X_STEP_M

        # 4) Stop condition: 모든 토픽 이미지가 소진되었으면 종료
        if all_done:
            self.get_logger().info("모든 폴더의 이미지를 전송 완료했습니다. 노드를 종료합니다.")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = MultiImageAndOdomPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()

