from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('arm_description'), 'urdf', 'arm.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('arm_bringup'), 'config', 'rviz_config.rviz')
    ros2_control_yaml_path = os.path.join(get_package_share_directory('arm_description'), 'config', 'ros2_control.yaml')

    urdf = xacro.process_file(urdf_path).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ros2_control_yaml_path]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager'],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager-timeout',
            '300',
            '--controller-manager',
            '/controller_manager'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(rviz_node)

    return ld
