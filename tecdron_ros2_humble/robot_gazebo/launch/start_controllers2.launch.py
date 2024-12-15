#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tecdron_base_controller"],
        remappings=[("/cmd_vel_unstamped", "/cmd_vel")],
        output="screen",
    )

    spawn_visual_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tecdron_visual_controller"],
        remappings=[("/cmd_vel_unstamped", "/cmd_vel")],
        output="screen",
    )
    

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_broadcaster,
            spawn_wheel_controller,
            spawn_visual_wheel_controller,

            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="robot_description",
                        executable="velocity_stamped",
                        name="velocity_stamped",
                        output="screen",
                    )   
                ],
            )
        ]
    )