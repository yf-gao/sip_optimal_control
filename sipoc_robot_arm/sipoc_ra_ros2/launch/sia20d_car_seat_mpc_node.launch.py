# Copyright (C) 2025 Robert Bosch GmbH

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# -------------------------------------
# Author: Yunfan Gao
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Robot URDF
    urdf_path = os.path.join(get_package_share_directory("sipoc_ra_support"), 'urdf', 'sia20d_car_seat.urdf')
    print(f"Loading URDF from: {urdf_path}")
    with open(urdf_path, 'r') as f:
        urdf_contents = f.read()
    robot_state_publisher = TimerAction(
        period = 2.0,
        actions = [
            Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_contents}],
            )
        ]
    )

    # Octomap Server for visualizing the environment
    octomap_path = os.path.join(get_package_share_directory("sipoc_ra_support"), 'meshes', 'car_seat', 'car_whole.bt')
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{'resolution': 0.05},
                    {'frame_id': "car"},
                    {'octomap_path': octomap_path}],
    )

    # sipoc_ra node
    ocp_node = TimerAction(
        period = 4.0,
        actions = [
            Node(
                package='sipoc_ra_ros2',
                executable='sia20d_car_seat_mpc_node',
                name='sia20d_car_seat_mpc_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('sipoc_ra_ros2'), 'params', 'sipoc_ra.yaml')],
            )
        ]
    )

    map_world_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'map'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # RViz for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('sipoc_ra_ros2'), 'rviz', 'sipoc_ra.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        octomap_server,
        ocp_node,
        map_world_static_tf,
        rviz2
    ])
