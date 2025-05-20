# launch/ball_follower_with_gazebo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_project',
            executable='ball_follower',
            name='ball_follower'
        ),
    ])
