from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='showimage',
            name='show_camera',
            output='screen',
            remappings=[
                ('/image', '/camera/image_raw'),
            ],
        ),
    ])
