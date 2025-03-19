#!/usr/bin/env python3
import rclpy
import csv
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CSVPosePublisher(Node):
    def __init__(self):
        super().__init__('csv_pose_publisher')
        # Create a publisher to send target poses.
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.file_path = "/home/kauf/Downloads/data.csv"  # Adjust as needed.
        self.timer = self.create_timer(2.0, self.publish_next_pose)
        try:
            self.csv_file = open(self.file_path, newline='')
            self.csv_reader = csv.reader(self.csv_file)
            self.get_logger().info(f"Opened CSV file: {self.file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {self.file_path}")
            self.csv_reader = iter([])

    def publish_next_pose(self):
        try:
            row = next(self.csv_reader)
        except StopIteration:
            self.get_logger().info("Reached end of CSV file.")
            self.destroy_timer(self.timer)
            self.csv_file.close()
            return
        if len(row) < 3:
            self.get_logger().warn(f"Invalid row: {row}")
            return
        try:
            x, y, z = map(float, row[:3])
        except ValueError:
            self.get_logger().warn(f"Invalid data: {row}")
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Neutral orientation.

        self.publisher_.publish(pose)
        self.get_logger().info(f"Published target pose: ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)
    node = CSVPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
