import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    full_path = os.path.join(get_package_share_directory('arm_description'), "urdf", "arm.urdf.xacro")
    rviz_path = os.path.join(get_package_share_directory('arm_description'), "rviz", "display.rviz")
    xacro = ParameterValue(Command(['xacro ', full_path]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': xacro}]
    )

    publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path]
    )

    arm_kinematics = Node(
        package='arm_kinematics',
        executable='ik_node'
    )

    marker_node = Node(
        package='arm_kinematics',
        executable='marker_node'
    )

    hardware = Node(
        package='arm_hardware',
        executable='hardware_node'
    )

    return LaunchDescription([rsp, #publisher, 
                              rviz, arm_kinematics, marker_node, hardware])