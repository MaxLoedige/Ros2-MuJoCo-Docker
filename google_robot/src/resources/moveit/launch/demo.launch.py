# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("google_robot", package_name="google_robot_moveit")
        .robot_description(
            file_path="config/google_robot.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": "mujoco"
            },
        )
        .robot_description_semantic(file_path="config/google_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # print(tmp.to_moveit_configs())
    # moveit_config = tmp.to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}]
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("google_robot_moveit"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ],
    )

    # Static TF
    # world2robot_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    #     parameters=[{"use_sim_time": True}]
    # )

    odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link",
        output="log",
        arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
        # arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        parameters=[{"use_sim_time": True}]
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("google_robot_moveit"),
        "config",
        "ros2_controllers.yaml",
    )

    # node_mujoco_ros2_control = Node(
    #     package='mujoco_ros2_control',
    #     executable='mujoco_ros2_control',
    #     output='screen',
    #     parameters=[
    #         moveit_config.robot_description,
    #         ros2_controllers_path,
    #         {'mujoco_model_path':os.path.join(get_package_share_directory('google_robot_mujoco'), 'google_robot', 'kitchen_scene.xml')},
    #         {"use_sim_time": True}
    #     ],
    #     prefix=['xterm -e gdb -ex run --args']
    # )

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {'mujoco_model_path':os.path.join(get_package_share_directory('google_robot_mujoco'), 'google_robot', 'scene.xml')},
            {"use_sim_time": True}
        ],
        # prefix=['xterm -e gdb -ex run --args']
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    google_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["google_arm_controller", "-c", "/controller_manager"],
    )

    google_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["google_hand_controller", "-c", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )


    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_mujoco_ros2_control,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            # admittance_controller_spawner
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[google_arm_controller_spawner, google_hand_controller_spawner, diff_drive_spawner],
                )
            ),
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=admittance_controller_spawner,
            #         on_exit=[google_arm_controller_spawner],
            #     )
            # ),
            rviz_node,
            odom_tf_node,
            robot_state_publisher,
            move_group_node,
            # joint_state_broadcaster_spawner
            node_mujoco_ros2_control
        ]
    )

    # return generate_demo_launch(moveit_config)
