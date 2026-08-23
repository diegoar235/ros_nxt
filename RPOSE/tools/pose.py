#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Intentamos importar el tipo del mensaje que usa CenterPose.
# Si falla, usamos "AnyMsg" y mostramos error claro.
try:
    from isaac_ros_pose_estimation_interfaces.msg import Detection3DArray  # a veces existe así
    MSG_TYPE_OK = True
except Exception:
    try:
        from vision_msgs.msg import Detection3DArray  # alternativa común
        MSG_TYPE_OK = True
    except Exception:
        MSG_TYPE_OK = False
        Detection3DArray = None


def _get_attr(obj, names):
    """Devuelve el primer atributo existente de 'names' dentro de obj, o None."""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None


def _extract_pose(msg):
    """
    Devuelve (header_frame_id, position(x,y,z), orientation(x,y,z,w)) o None.
    Navega estructuras comunes:
      msg.detections / msg.results / msg.objects
      cada item: .results[0].pose.pose / .pose / .bbox.center / etc.
    """
    header = _get_attr(msg, ["header"])
    frame_id = ""
    if header is not None and hasattr(header, "frame_id"):
        frame_id = header.frame_id or ""

    candidates = _get_attr(msg, ["detections", "results", "objects"])
    if not candidates:
        return None

    first = candidates[0]

    # Algunos mensajes tienen "results" dentro de cada detection
    inner_results = _get_attr(first, ["results"])
    if inner_results:
        first = inner_results[0]

    # Pose puede estar en varias ubicaciones:
    pose = _get_attr(first, ["pose", "pose_with_covariance", "pose_with_covariance_stamped"])
    if pose is None:
        # A veces está como pose.pose
        pose = _get_attr(first, ["pose_stamped"])
    # Desenrollar pose.pose si aplica
    if pose is not None and hasattr(pose, "pose"):
        pose = pose.pose

    # Algunos formatos: pose.position / pose.orientation
    if pose is None:
        # alternativa: bbox.center es Pose
        bbox = _get_attr(first, ["bbox", "bounding_box"])
        if bbox is not None:
            center = _get_attr(bbox, ["center"])
            if center is not None:
                pose = center

    if pose is None:
        return None

    pos = _get_attr(pose, ["position"])
    ori = _get_attr(pose, ["orientation"])
    if pos is None or ori is None:
        return None

    return frame_id, (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w)


class TfFromCenterPose(Node):
    def __init__(self):
        super().__init__("tf_from_centerpose")
        if not MSG_TYPE_OK:
            self.get_logger().error(
                "No pude importar Detection3DArray (ni isaac_ros_pose_estimation_interfaces.msg ni vision_msgs.msg).\n"
                "Decime qué tipo muestra: `ros2 topic info /centerpose/detections -v` y lo ajusto."
            )
            raise RuntimeError("Missing message type for /centerpose/detections")

        self.declare_parameter("detections_topic", "/centerpose/detections")
        self.declare_parameter("parent_fallback", "camera_frame")
        self.declare_parameter("child_frame", "shoe")

        topic = self.get_parameter("detections_topic").value
        self.parent_fallback = self.get_parameter("parent_fallback").value
        self.child_frame = self.get_parameter("child_frame").value

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Detection3DArray, topic, self.cb, 10
        )

        self.get_logger().info(f"Escuchando {topic} y publicando TF child='{self.child_frame}'")

    def cb(self, msg):
        out = _extract_pose(msg)
        if out is None:
            # No spamear demasiado
            self.get_logger().warn("No pude extraer pose de este mensaje (estructura inesperada).", throttle_duration_sec=2.0)
            return

        frame_id, (x, y, z), (qx, qy, qz, qw) = out
        parent = frame_id if frame_id else self.parent_fallback

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = self.child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = TfFromCenterPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()