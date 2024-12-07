#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from drivers import DisparityExtenderStable  # Import your motion planner

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("path_publisher")

        # Declare and get parameters
        self.declare_parameter("control_topic", "/mux/ackermann_cmd_mux/input/navigation")
        control_topic = self.get_parameter("control_topic").get_parameter_value().string_value

        self.planner = DisparityExtenderStable()

        # Publisher for controls
        self.pub_controls = self.create_publisher(AckermannDriveStamped, control_topic, 10)

        # Subscriber for LaserScan
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.get_logger().info("PathPublisherNode initialized!")

    def scan_callback(self, msg):
        scan = msg.ranges
        self.get_logger().info("------------------------SCAN------------------------")
        self.get_logger().info(f"{scan}")
        self.get_logger().info("------------------------SCAN------------------------")

        speed, steer = self.planner.process_lidar(scan)

        res = AckermannDriveStamped()
        res.drive = AckermannDrive()
        res.drive.steering_angle = steer
        res.drive.speed = speed

        self.pub_controls.publish(res)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
