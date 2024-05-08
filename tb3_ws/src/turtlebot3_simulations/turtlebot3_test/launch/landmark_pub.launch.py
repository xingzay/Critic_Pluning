from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Nodes

    # pub_landmark = Node(
    #     package='trilateration',
    #     executable='pub_landmark',
    #     # prefix=["gdbserver localhost:3001"],
    #     output='screen',
    # )
    check_turtlebot3_node = Node(
        package='turtlebot3_test',
        executable='check_turtlebot3_node',
        # prefix=["gdbserver localhost:3001"],
        output='screen',
    )
    return LaunchDescription([
        # Nodes
        # pub_landmark,
        check_turtlebot3_node
    ])
