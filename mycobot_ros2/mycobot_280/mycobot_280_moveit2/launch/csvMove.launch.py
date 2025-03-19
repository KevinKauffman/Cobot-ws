#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Build the MoveIt2 configuration using your URDF.
    moveit_config_builder = MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2")
    moveit_config_builder.robot_description_command = "xacro " + os.path.join(
        get_package_share_directory("mycobot_description"),
        "urdf", "mycobot_280_m5", "mycobot_280_m5.urdf"
    )
    moveit_config = moveit_config_builder.to_moveit_configs()

    # Launch the MoveIt2 move_group node using the helper.
    move_group_launch = generate_move_group_launch(moveit_config)
    # Launch RViz.
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    # Launch the CSV Publisher node.
    csv_node = Node(
        package='mycobot_280_moveit2',
        executable='csv_publisher',  # Ensure this is the installed executable name.
        name='csv_publisher',
        output='screen',
        parameters=[{'file_path': '/home/kauf/Downloads/data.csv'}]
    )

    # Launch the Planning Bridge node with a delay.
    planning_bridge_node = TimerAction(
        period=5.0,  # Delay to allow move_group to initialize.
        actions=[
            Node(
                package='mycobot_280_moveit2',
                executable='planning_bridge',  # Ensure this is the installed executable name.
                name='planning_bridge',
                output='screen',
                remappings=[('move_group', '/move_group')]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(move_group_launch)
    ld.add_action(rviz_launch)
    ld.add_action(csv_node)
    ld.add_action(planning_bridge_node)
    return ld

if __name__ == '__main__':
    generate_launch_description()
