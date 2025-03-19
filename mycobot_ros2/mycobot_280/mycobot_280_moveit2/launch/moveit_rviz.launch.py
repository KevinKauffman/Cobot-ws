#!/usr/bin/env python3
import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create a builder for the "firefighter" planning group.
    moveit_config_builder = MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2")

    # Override the robot_description_command to use your proper URDF.
    robot_description_command = "xacro " + os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf", "mycobot_280_m5", "mycobot_280_m5.urdf"
    )
    moveit_config_builder.set_robot_description_command(robot_description_command)

    # Generate the full MoveIt2 configuration.
    moveit_config = moveit_config_builder.to_moveit_configs()

    # Return the RViz launch description built from the configuration.
    return generate_moveit_rviz_launch(moveit_config)
