import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open rviz'
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]

    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='so_arm_100', package_name='so_arm_100_moveit_config')
        .robot_description_semantic(
            str(Path('config') / 'so_arm_100.srdf'))
        .joint_limits(str(Path('config') / 'joint_limits.yaml'))
        .trajectory_execution(
            str(Path('config') / 'moveit_controllers.yaml'))
        .robot_description_kinematics(
            str(Path('config') / 'kinematics.yaml'))
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('so_arm_100_moveit_config'), 'config', 'moveit.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(use_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        arguments
        + [
            move_group_node,
            rviz_node,
        ]
    )
