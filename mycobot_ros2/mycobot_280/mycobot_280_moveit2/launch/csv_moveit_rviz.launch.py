#import os
#from launch import LaunchDescription
#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch_ros.actions import Node
#from ament_index_python.packages import get_package_share_directory

#def generate_launch_description():
    ## Include the MoveIt2 demo launch. This launch file (demo.launch.py)
    ## from your mycobot_280_moveit2 package should start the move_group node,
    ## RViz, and any other necessary nodes.
    #moveit_demo = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource(
            #os.path.join(
                #get_package_share_directory('mycobot_280_moveit2'),
                #'launch',
                #'demo.launch.py'
            #)
        #)
    #)

    ## Launch the CSV Pose Publisher node (your CSV control node)
    ## This node will read target poses from a CSV file and publish them
    ## (for example, on the topic '/target_pose').
    #csv_node = Node(
        #package='mycobot_280_moveit2_control',
        #executable='csv_stream_control',
        #name='csv_pose_publisher',
        #output='screen',
        #parameters=[{'file_path': '/home/kauf/Downloads/data.csv'}]  # Adjust the path if needed
    #)

    #return LaunchDescription([
        #moveit_demo,
        #csv_node,
    #])

