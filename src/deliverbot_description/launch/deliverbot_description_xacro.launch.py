import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="deliverbot_description",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="deliverbot.xacro",
            description="k-position factor in the safety controller.",
        )
    )

    # Initialize Arguments
    #use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    #description_package = LaunchConfiguration("description_package")
    #description_file = LaunchConfiguration("description_file")
    #fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    robot_urdf_file = os.path.join(get_package_share_directory(
    'deliverbot_description'), 'urdf', 'deliverbot.xacro')
    robot_urdf = xacro.process_file(robot_urdf_file)
    robot_description = {'robot_description': robot_urdf.toxml()}

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(
        'deliverbot_description'), "rviz", "deliverbot.rviz"]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    # )
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        # joint_state_publisher_node,
        # robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
