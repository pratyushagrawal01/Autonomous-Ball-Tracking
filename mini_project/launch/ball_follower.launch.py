# launch/ball_follower.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to your robot's URDF file (no need for xacro now)
    urdf_file_path = '/home/alakh/ros2_ws/src/mini_project/urdf/four_wheeled_robot.urdf'

    # Read the URDF file and load it as a string for the robot_description parameter
    with open(urdf_file_path, 'r') as infp:
        robot_description = infp.read()

    # Launch the robot_state_publisher to publish the URDF to the /robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'ball_follower_robot'],
        output='screen'
    )

    return LaunchDescription([
        # Launch the robot_state_publisher to publish the URDF
        robot_state_publisher,
        # Launch Gazebo
        gazebo,
        spawn_entity
    ])
