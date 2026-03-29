"""Launch the full dual-UR5e Gazebo + web demo."""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
        ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ──────────────────────────────────────────────────────
    pkg        = get_package_share_directory("ur5e_collision_avoidance")
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    world_file   = os.path.join(pkg, "worlds", "ur5e_workspace.world")
    xacro_file   = os.path.join(pkg, "urdf",   "dual_ur5e.urdf.xacro")
    ctrl_config  = os.path.join(pkg, "config",  "dual_ur5e_controllers.yaml")
    avoid_config = os.path.join(pkg, "config",  "collision_avoidance_params.yaml")
    rviz_config  = os.path.join(pkg, "rviz",    "dual_ur5e.rviz")

    # ROS2 Humble Python packages target Python 3.10.
    # If conda is active (e.g. Python 3.13), rclpy import can fail.
    ros_py_env = {
        "PATH": "/usr/bin:/bin:/opt/ros/humble/bin:" + os.environ.get("PATH", ""),
        "PYTHONPATH": (
            "/opt/ros/humble/lib/python3.10/site-packages:"
            "/opt/ros/humble/local/lib/python3.10/dist-packages"
        ),
    }

    # ── Launch args ────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2")
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Show Gazebo GUI")

    use_rviz = LaunchConfiguration("use_rviz")
    gui      = LaunchConfiguration("gui")

    # ── Robot description ─────────────────────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " sim_gazebo:=true",
        " use_fake_hardware:=false",
        " fake_sensor_commands:=false",
        " simulation_controllers:=", ctrl_config,
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── 1. Gazebo ──────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world":   world_file,
            "gui":     gui,
            "verbose": "false",
            "pause":   "false",
        }.items(),
    )

    # ── 2. robot_state_publisher ───────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── 3. Spawn robot into Gazebo (after 2 s so Gazebo is ready) ─
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_dual_ur5e",
                arguments=[
                    "-topic", "/robot_description",
                    "-entity", "dual_ur5e",
                    "-x", "0.0", "-y", "0.0", "-z", "0.0",
                ],
                output="screen",
                additional_env=ros_py_env,
            )
        ],
    )

    # ── 4. ros2_control node ───────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ctrl_config],
        output="screen",
        additional_env=ros_py_env,
    )

    # ── 5-6. Spawners (3 s after ros2_control starts) ─────────────
    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
            additional_env=ros_py_env,
        )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=3.0, actions=[
                    spawner("robot1_joint_state_broadcaster"),
                    spawner("robot2_joint_state_broadcaster"),
                    spawner("robot1_arm_controller"),
                    spawner("robot2_arm_controller"),
                ])
            ],
        )
    )

    # ── 7-8. Avoidance nodes ───────────────────────────────────────
    collision_monitor = Node(
        package="ur5e_collision_avoidance",
        executable="collision_monitor.py",
        name="collision_monitor",
        parameters=[avoid_config],
        output="screen",
        additional_env=ros_py_env,
    )
    robot1_ctrl = Node(
        package="ur5e_collision_avoidance",
        executable="robot1_controller.py",
        name="robot1_controller",
        output="screen",
        additional_env=ros_py_env,
    )
    robot2_ctrl = Node(
        package="ur5e_collision_avoidance",
        executable="robot2_controller.py",
        name="robot2_controller",
        output="screen",
        additional_env=ros_py_env,
    )

    # ── 9. Scenario orchestrator (8 s delay) ──────────────────────
    orchestrator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur5e_collision_avoidance",
                executable="scenario_orchestrator.py",
                name="scenario_orchestrator",
                parameters=[avoid_config, {"auto_start": False}],
                output="screen",
                additional_env=ros_py_env,
            )
        ],
    )

    # ── 10. RViz2 ─────────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # ROS <-> WebSocket bridge used by the HTML dashboard.
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090, "address": ""}],
        additional_env=ros_py_env,
    )

    # HTTP + WS proxy server for the dashboard on localhost:8000.
    web_server = ExecuteProcess(
        cmd=[sys.executable, os.path.join(pkg, "rviz", "web_server.py")],
        output="screen",
        additional_env={
            **ros_py_env,
            "WEB_PORT": "8000",
            "ROSBRIDGE_URL": "ws://localhost:9090",
            "RVIZ_DIR": os.path.join(pkg, "rviz"),
        },
    )

    return LaunchDescription([
        use_rviz_arg,
        gui_arg,
        gazebo,
        robot_state_publisher,
        ros2_control_node,
        spawn_robot,
        load_controllers,
        collision_monitor,
        robot1_ctrl,
        robot2_ctrl,
        orchestrator,
        rviz2,
        rosbridge,
        web_server,
    ])