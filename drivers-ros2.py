#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
from drivers import DisparityExtenderStable  # Import your motion planner

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("path_publisher")

        self.declare_parameter('vel_max', 5.0)
        self.declare_parameter('min_vel_ratio', 0.3)

        # Declare and get parameters
        self.declare_parameter("throttle_topic", "/autodrive/f1tenth_1/throttle_command")
        self.declare_parameter("steering_topic", "/autodrive/f1tenth_1/steering_command")

        throttle_topic = self.get_parameter("throttle_topic").get_parameter_value().string_value
        steering_topic = self.get_parameter("steering_topic").get_parameter_value().string_value

        self.vel_max = float(self.get_parameter('vel_max').get_parameter_value().double_value)
        self.min_vel_ratio = float(self.get_parameter('min_vel_ratio').get_parameter_value().double_value)
        self.angle_limit = np.pi / 6
        self.integral = 0.0
        self.kp = 0.45
        self.ki = 0.05
        self.kd = 0.1
        self.prev_error=0.0
        self.time_interval = 0.1

        self.planner = DisparityExtenderStable()

        # Publishers for speed and steering
        self.pub_speed = self.create_publisher(Float32, throttle_topic, 10)
        self.pub_steering = self.create_publisher(Float32, steering_topic, 10)

        # Subscriber for LaserScan
        self.sub_scan = self.create_subscription(LaserScan, "/autodrive/f1tenth_1/lidar", self.scan_callback, 10)

        self.get_logger().info("PathPublisherNode initialized!")

    def scan_callback(self, msg):
        scan = msg.ranges
        self.get_logger().info("------------------------SCAN------------------------")
        self.get_logger().info(f"{scan}")
        self.get_logger().info("------------------------SCAN------------------------")

        # Process LiDAR data with planner
        speed, steer = self.planner.process_lidar(scan)
        steer = max(min(steer, self.angle_limit), -self.angle_limit)
        steering_msg = Float32()
        steering_msg.data = steer
        self.pub_steering.publish(steering_msg)

        # Publish speed
        throttle_msg = Float32()
        self.target_velocity=self.vel_max*(1-(((1-self.min_vel_ratio)*abs(steer))/self.angle_limit))
        error = self.target_velocity - speed
        derivative = (error - self.prev_error) / self.time_interval
        self.integral += error * self.time_interval
        throttle = self.kp * error + self.kd * derivative
        throttle = max(-1.0, min(1.0, throttle))
        self.prev_error = error
        throttle_msg.data = speed
        self.pub_speed.publish(throttle)
        

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

    main()
