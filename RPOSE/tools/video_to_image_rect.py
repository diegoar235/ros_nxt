#!/usr/bin/env python3
import math
import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def make_camera_info(width: int, height: int, frame_id: str, fov_deg: float) -> CameraInfo:
    # Simple pinhole intrinsics from assumed horizontal FOV
    fov_rad = math.radians(fov_deg)
    fx = (width / 2.0) / math.tan(fov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height

    # Distortion (assume none)
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    # K (3x3)
    msg.k = [
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    ]

    # R (identity)
    msg.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    ]

    # P (3x4)
    msg.p = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0
    ]

    return msg


class VideoPublisher(Node):
    def __init__(self, video_path: str, fps: float, fov_deg: float,
                 image_topic: str, info_topic: str, frame_id: str, loop: bool):
        super().__init__("video_to_image_rect")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"No pude abrir el video: {video_path}")


        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_pub = self.create_publisher(Image, image_topic, qos)
        self.info_pub  = self.create_publisher(CameraInfo, info_topic, qos)


        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("El video no tiene frames.")
        self.height, self.width = frame.shape[:2]
        self.camera_info = make_camera_info(self.width, self.height, frame_id, fov_deg)

        # volver al inicio
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.frame_id = frame_id
        self.loop = loop
        self.timer = self.create_timer(1.0 / fps, self.tick)

        self.get_logger().info(
            f"Publicando {video_path} a {image_topic} y {info_topic} @ {fps} Hz "
            f"({self.width}x{self.height}, fov={fov_deg}deg, loop={loop})"
        )

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
                if not ok:
                    self.get_logger().error("No pude reiniciar el video.")
                    return
            else:
                self.get_logger().info("Fin del video (loop desactivado).")
                rclpy.shutdown()
                return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id

        info_msg = self.camera_info
        info_msg.header.stamp = now  # importante: timestamp alineado

        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True, help="Ruta al video (dentro del contenedor)")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--fov", type=float, default=70.0, help="FOV horizontal estimado en grados")
    parser.add_argument("--image_topic", default="/image_rect")
    parser.add_argument("--info_topic", default="/camera_info_rect")
    parser.add_argument("--frame_id", default="camera")
    parser.add_argument("--no_loop", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = VideoPublisher(
        video_path=args.video,
        fps=args.fps,
        fov_deg=args.fov,
        image_topic=args.image_topic,
        info_topic=args.info_topic,
        frame_id=args.frame_id,
        loop=not args.no_loop
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
