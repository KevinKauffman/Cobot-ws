#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    PlanningOptions,
    RobotState
)
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient

def create_motion_plan_request(target_pose: PoseStamped, group_name: str, ee_link: str) -> MotionPlanRequest:
    req = MotionPlanRequest()
    req.group_name = group_name
    req.allowed_planning_time = 5.0
    # Provide a default (empty) start state.
    req.start_state = RobotState()

    constraints = Constraints()

    # Create a PositionConstraint.
    pos_constraint = PositionConstraint()
    pos_constraint.header = target_pose.header
    pos_constraint.link_name = ee_link
    pos_constraint.target_point_offset.x = 0.0
    pos_constraint.target_point_offset.y = 0.0
    pos_constraint.target_point_offset.z = 0.0

    # Define an acceptable region around the target (a sphere with 5 cm radius).
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.05]
    pos_constraint.constraint_region.primitives.append(primitive)
    pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

    # Create an OrientationConstraint.
    ori_constraint = OrientationConstraint()
    ori_constraint.header = target_pose.header
    ori_constraint.link_name = ee_link
    ori_constraint.orientation = target_pose.pose.orientation
    ori_constraint.absolute_x_axis_tolerance = 0.1
    ori_constraint.absolute_y_axis_tolerance = 0.1
    ori_constraint.absolute_z_axis_tolerance = 0.1
    ori_constraint.weight = 1.0

    constraints.position_constraints.append(pos_constraint)
    constraints.orientation_constraints.append(ori_constraint)
    req.goal_constraints.append(constraints)
    return req

class PlanningBridge(Node):
    def __init__(self):
        super().__init__('planning_bridge')
        # Subscribe to the PoseStamped target pose.
        self.subscription = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_pose_callback,
            10)
        # Create an action client for the MoveGroup action server.
        self._action_client = ActionClient(self, MoveGroup, 'move_group')
        # Use the planning group from your SRDF and assume the end-effector link.
        self.group_name = "arm_group"        # from your SRDF
        self.ee_link = "joint6_flange"       # adjust if your configuration differs

    def target_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received target pose: {msg}")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return

        # Create a MotionPlanRequest using the target pose.
        request = create_motion_plan_request(msg, self.group_name, self.ee_link)
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request

        # Set planning options to plan only for debugging.
        options = PlanningOptions()
        options.plan_only = True  # Change to False once planning works reliably.
        options.replan = False
        goal_msg.planning_options = options

        self.get_logger().info(f"Sending goal: {goal_msg}")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup action server!")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = PlanningBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
