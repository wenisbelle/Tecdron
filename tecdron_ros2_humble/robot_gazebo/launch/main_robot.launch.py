import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the launch files
    gazebo_launch_file = os.path.join(
        get_package_share_directory('robot_gazebo'), 'launch', 'start_world.launch.py')
    
    urdf_launch_file = os.path.join(
        get_package_share_directory('robot_description'), 'launch', 'urdf_visualize.launch.py')
    
    spawn_robot_launch_file = os.path.join(
        get_package_share_directory('robot_gazebo'), 'launch', 'spawn_robot.launch.py')
    
    spawn_controllers_launch_file = os.path.join(
        get_package_share_directory('robot_gazebo'), 'launch', 'start_controllers.launch.py')
    

    return LaunchDescription([
        # Launch first_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={}.items(),
        ),

        # Delay before launching the robot_state_publisher
        TimerAction(
            period=10.0,  # Adjust the delay time as needed
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(urdf_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),
        # Delay before launching the spawner
        TimerAction(
            period=5.0,  
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_robot_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),

        TimerAction(
            period=5.0, 
            actions=[
                # Launch second_launch.py
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_controllers_launch_file),
                    launch_arguments={}.items(),
                ),
            ],
        ),

    ])
