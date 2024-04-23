import os
import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():

    ld = LaunchDescription()
    simulation_launch_dir = os.path.join(
        get_package_share_directory('deliverbot_gazebo'), 'launch')
    
    package_name = 'deliverbot_description'
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    gazebo_world_path = os.path.join(pkg_share, 'world/house4_3.world')
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')



    use_sim_time = LaunchConfiguration('use_sim_time') # type: ignore
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub') # type: ignore

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # urdf_file = os.path.join(get_package_share_directory(
    #     'deliverbot_description'), 'urdf', 'deliverbot.xacro')
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # robot_description_config = doc.toxml()
    # robot_description = {'robot_description': robot_description_config}
    robot_urdf_file = os.path.join(get_package_share_directory(
    'deliverbot_description'), 'urdf', 'deliverbot.xacro')
    robot_urdf = xacro.process_file(robot_urdf_file)
    robot_description = {'robot_description': robot_urdf.toxml()}


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'deliverbot'],
                        output='screen')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub), # type: ignore
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description])
    
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     output='screen',
    # )
    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(
        'deliverbot_description'), "rviz", "deliverbot1.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(spawn_entity)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    # ld.add_action(joint_state_publisher_node)
    # ld.add_action(rviz_node)

    return ld
