import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description(context, *args, **kwargs):
    so_arm_100_description_path = os.path.join(
        get_package_share_directory('so_arm_100_description')
    )

    dof = LaunchConfiguration('dof').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    xacro_file = os.path.join(
        so_arm_100_description_path,
        'urdf',
        f'so_arm_100_{dof}dof.urdf.xacro',
    )

    doc = xacro.process_file(xacro_file, mappings={
        'prefix': prefix,
        'use_sim': 'false',
        'use_fake_hardware': 'true',
    })

    robot_desc = doc.toprettyxml(indent='  ')

    rviz_config_file = os.path.join(
        so_arm_100_description_path,
        'config',
        'so_arm_100.rviz'
    )

    return {
        'robot_description': ParameterValue(robot_desc, value_type=str),
        'rviz_config_file': rviz_config_file,
    }

def generate_launch_description():
    # Launch arguments
    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'dof',
            default_value='5',
            description='DOF configuration - either 5 or 7'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of joint and link names'
        ),
    ])

    def launch_setup(context, *args, **kwargs):
        params = get_robot_description(context)
        
        nodes = [
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': params['robot_description']}],
                name='robot_state_publisher'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', params['rviz_config_file']]
            )
        ]
        return nodes

    return LaunchDescription([
        arguments,
        OpaqueFunction(function=launch_setup),
    ])
