#!/usr/bin/env python3
import rclpy
import csv
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CSVPosePublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.declare_parameter('file_path', '/home/kauf/Downloads/data.csv')
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value

        try:
            self.csv_file = open(self.file_path, newline='')
            self.csv_reader = csv.reader(self.csv_file)
            self.get_logger().info(f"Opened CSV file: {self.file_path}")
        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found at: {self.file_path}")
            self.csv_reader = iter([])

        # Publish a new pose every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_next_pose)

    def publish_next_pose(self):
        try:
            row = next(self.csv_reader)
        except StopIteration:
            self.get_logger().info("Reached end of CSV file. Shutting down publisher.")
            self.csv_file.close()
            self.destroy_timer(self.timer)
            return

        if len(row) < 3:
            self.get_logger().warn(f"Skipping invalid row: {row}")
            return

        try:
            # Convert from millimeters to meters
            x, y, z = [float(val) / 1000.0 for val in row[:3]]
        except ValueError:
            self.get_logger().warn(f"Skipping row with invalid data: {row}")
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Use a neutral orientation (modify if needed)
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Publishing target pose: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = CSVPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

