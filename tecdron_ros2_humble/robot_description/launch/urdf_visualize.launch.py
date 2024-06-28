import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import launch_ros.descriptions

import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")

    
    robot_model_path = os.path.join(
        get_package_share_directory('robot_description'))
    xacro_file = os.path.join(robot_model_path, 'urdf', 'tecdron.xacro')

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
        ]
    )