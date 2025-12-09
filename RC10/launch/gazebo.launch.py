from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('RC10')
    urdf_file = os.path.join(pkg_share, 'urdf', 'RC10.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Запуск пустого мира Gazebo (classic) из gazebo_ros
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    # Статические трансформации
    static_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen',
    )

    static_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )

    # Спавн сущности в Gazebo из /robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=['-topic', 'robot_description', '-entity', 'RC10'],
        output='screen',
    )

    return LaunchDescription(
        [
            gazebo,
            static_map_to_base,
            static_footprint_base,
            robot_state_publisher_node,
            spawn_entity,
        ]
    )


