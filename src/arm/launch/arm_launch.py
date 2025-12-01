from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('arm_description'), 'urdf', 'arm.urdf.xacro')
    ros2_control_yaml_path = os.path.join(get_package_share_directory('arm_description'), 'config', 'ros2_control.yaml')

    urdf = xacro.process_file(urdf_path).toxml()

    fox_glove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )

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

    inverse_kinematics_service_node = Node(
        package='inverse_kinematics',
        executable='ik_service_node',
        output='screen'
    )

    pose_to_joint_trajectory_node = Node(
        package='arm',
        executable='pose_to_joint_trajectory_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(fox_glove_bridge_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(inverse_kinematics_service_node)
    ld.add_action(pose_to_joint_trajectory_node)

    return ld
