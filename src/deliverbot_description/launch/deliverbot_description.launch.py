# Copyright 2019 Carlos San Vicente
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the bringup directory
    bringup_dir = FindPackageShare('deliverbot_description').find('deliverbot_description')

    # Set robot description parameters
    urdf_file = os.path.join(bringup_dir, 'urdf', 'deliverbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    # Set rviz config path
    rviz_cfg_path = os.path.join(bringup_dir, 'rviz/deliverbot.rviz')

    with_rviz_param = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RVIZ2 in addition to other nodes'
    )

    #Node definitions
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params],
        condition = IfCondition(LaunchConfiguration('rviz'))
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rviz',default_value='true',
                                            description='Flag to enable rviz'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        with_rviz_param,
        rviz_node
        ])

