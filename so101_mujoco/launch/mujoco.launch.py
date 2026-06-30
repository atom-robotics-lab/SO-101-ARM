import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with MoveIt plugin",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("so_arm_100_moveit_config"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
            description="Path to initial joint positions YAML",
        ),
        DeclareLaunchArgument(
            "MCJFFile",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("so101_mujoco"),
                    "scene.xml",
                ]
            ),
            description="Path to initial joint positions YAML",
        ),
    ]

    use_rviz = LaunchConfiguration("rviz")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    mcjf_file = LaunchConfiguration("MCJFFile")

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("so_arm_100_description"),
                        "urdf",
                        "so_arm_100_5dof.urdf.xacro",
                    ]
                ),
                " use_sim:=true",
                " use_fake_hardware:=false",
                " use_topic_hardware_interface:=false",
                " initial_positions_file:=",
                initial_positions_file,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}


    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("so_arm_100_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[mcjf_file, controllers_yaml],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )




    moveit_config = (
        MoveItConfigsBuilder("so_arm_100", package_name="so_arm_100_moveit_config")
        .robot_description(
            file_path=str(
                Path(
                    os.popen(
                        "ros2 pkg prefix so_arm_100_description"
                    ).read().strip()
                )
                / "share"
                / "so_arm_100_description"
                / "urdf"
                / "so_arm_100_5dof.urdf.xacro"
            ),
            mappings={
                "use_sim": "true",
                "use_fake_hardware": "false",
                "use_topic_hardware_interface": "false",
            },
        )
        .robot_description_semantic(str(Path("config") / "so_arm_100.srdf"))
        .joint_limits(str(Path("config") / "joint_limits.yaml"))
        .trajectory_execution(str(Path("config") / "moveit_controllers.yaml"))
        .robot_description_kinematics(str(Path("config") / "kinematics.yaml"))
        .to_moveit_configs()
    )
 
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("so_arm_100_moveit_config"), "config", "moveit.rviz"]
    )
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    delayed_moveit = TimerAction(
        period=5.0,
        actions=[
            move_group_node,
            rviz_node,
        ],
    )




    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
 
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )
 
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )
 
    arm_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_effort_controller", "--inactive"],
        output="screen",
    )
 
    gripper_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_effort_controller", "--inactive"],
        output="screen",
    )

    delayed_spawners = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            arm_effort_controller_spawner,
            gripper_effort_controller_spawner,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            mujoco_ros2_control_node,
            delayed_spawners,
            delayed_moveit,
        ]
    )