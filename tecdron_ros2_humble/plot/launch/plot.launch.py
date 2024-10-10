from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'real_life',
            default_value='false',
            description='Set to true for real-life operation'
        ),
        DeclareLaunchArgument(
            'odom_topics',
            default_value="['odom_model']",
            description='List of odometry topics to subscribe to'
        ),
        DeclareLaunchArgument(
            'true_odom',
            default_value='true_odom',
            description='List of odometry topics to subscribe to'
        ),
        Node(
            package='plot',
            executable='plot',
            name='plot',
            parameters=[{
                'real_life': LaunchConfiguration('real_life'),
                'odom_topics': LaunchConfiguration('odom_topics'),
                'true_odom': LaunchConfiguration('true_odom')
            }],
            output='screen'
        )
    ])