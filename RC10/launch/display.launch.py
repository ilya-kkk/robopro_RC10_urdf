from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('RC10')
    urdf_file = os.path.join(pkg_share, 'urdf', 'RC10.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    static_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
    )

    rviz_config = os.path.join(pkg_share, 'urdf.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription(
        [
            static_map_to_base,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )


