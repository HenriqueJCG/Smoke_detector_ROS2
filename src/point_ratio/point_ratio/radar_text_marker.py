import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from std_msgs.msg import Float64


class RadarTextMarker(Node):

    def __init__(self):
        super().__init__("radar_text_marker")

        self._sensor_ratio = 1.0
        self._lidar_points = 1.0
        self._radar_points = 1.0

        self.create_subscription(
            Float64,
            "/point_ratio",
            self.callback,
            10
        )

        self.create_subscription(
            Float64,
            "/lidar_points",
            self.callback_lidar,
            10
        )

        self.create_subscription(
            Float64,
            "/radar_points",
            self.callback_radar,
            10
        )

        self.pub = self.create_publisher(
            Marker,
            "visualization_marker",
            10
        )

    def callback_lidar(self, msg):
        self._lidar_points = msg.data

    def callback_radar(self, msg):
        self._radar_points = msg.data

    def callback(self, msg):

        self._sensor_ratio = msg.data

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "radar_info"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0

        marker.scale.z = 0.5

        if self._sensor_ratio < 0.2:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

        elif self._sensor_ratio < 0.5:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

        marker.text = (
            f"Sensor Point Ratio: {self._sensor_ratio:.3f}"
            f"\nLidar Points: {self._lidar_points:.0f}"
            f"\nRadar Points: {self._radar_points:.0f}"
        )

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = RadarTextMarker()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()