"""
simulation.launch.py
====================
Master launch file.

What it starts
──────────────
  1. Gazebo  (ur5e_workspace.world)
  2. robot_state_publisher  (dual_ur5e.urdf.xacro → /robot_description)
  3. Spawn both UR5e models into Gazebo
  4. ros2_control controller_manager
  5. Joint state broadcasters × 2
  6. JointTrajectoryControllers × 2
  7. collision_monitor node
  8. robot1_controller  +  robot2_controller
  9. scenario_orchestrator
  10. RViz2 (optional, via launch arg use_rviz:=true)

Usage
─────
  ros2 launch ur5e_collision_avoidance simulation.launch.py
  ros2 launch ur5e_collision_avoidance simulation.launch.py use_rviz:=true
  ros2 launch ur5e_collision_avoidance simulation.launch.py gui:=false   # headless
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share   = get_package_share_directory("ur5e_collision_avoidance")
    ur_desc_dir = get_package_share_directory("ur_description")
    gazebo_dir  = get_package_share_directory("gazebo_ros")

    # ── Launch arguments ─────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Show Gazebo GUI window")
    rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Launch RViz2")
    verbose_arg = DeclareLaunchArgument(
        "verbose", default_value="false",
        description="Verbose Gazebo output")

    gui     = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")
    verbose  = LaunchConfiguration("verbose")

    # ── World & URDF paths ───────────────────────────────────
    world_file    = os.path.join(pkg_share, "worlds", "ur5e_workspace.world")
    xacro_file    = os.path.join(pkg_share, "urdf", "dual_ur5e.urdf.xacro")
    ctrl_config   = os.path.join(pkg_share, "config", "dual_ur5e_controllers.yaml")
    avoid_config  = os.path.join(pkg_share, "config", "collision_avoidance_params.yaml")
    rviz_config   = os.path.join(pkg_share, "rviz", "dual_ur5e.rviz")

    # ── Robot description via xacro ──────────────────────────
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        xacro_file, " ",
        "sim_gazebo:=true ",
        "simulation_controllers:=", ctrl_config,
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── 1. Gazebo ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world":   world_file,
            "gui":     gui,
            "verbose": verbose,
            "pause":   "false",
        }.items(),
    )

    # ── 2. robot_state_publisher ─────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── 3. Spawn URDF into Gazebo ────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_dual_ur5e",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "dual_ur5e",
            "-x", "0", "-y", "0", "-z", "0",
        ],
        output="screen",
    )

    # ── 4. Controller manager ────────────────────────────────
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ctrl_config],
        output="screen",
    )

    # ── 5-6. Spawning controllers (after controller_manager) ─
    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    spawn_r1_jsb = spawner("robot1_joint_state_broadcaster")
    spawn_r2_jsb = spawner("robot2_joint_state_broadcaster")
    spawn_r1_arm = spawner("robot1_arm_controller")
    spawn_r2_arm = spawner("robot2_arm_controller")

    # ── 7. Collision monitor ─────────────────────────────────
    collision_monitor = Node(
        package="ur5e_collision_avoidance",
        executable="collision_monitor.py",
        name="collision_monitor",
        parameters=[avoid_config],
        output="screen",
    )

    # ── 8. Robot controllers ─────────────────────────────────
    r1_ctrl = Node(
        package="ur5e_collision_avoidance",
        executable="robot1_controller.py",
        name="robot1_controller",
        output="screen",
    )
    r2_ctrl = Node(
        package="ur5e_collision_avoidance",
        executable="robot2_controller.py",
        name="robot2_controller",
        output="screen",
    )

    # ── 9. Scenario orchestrator (delayed 5s to let Gazebo settle) ──
    orchestrator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ur5e_collision_avoidance",
                executable="scenario_orchestrator.py",
                name="scenario_orchestrator",
                parameters=[avoid_config],
                output="screen",
            )
        ],
    )

    # ── 10. Optional RViz2 ───────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=LaunchConfiguration("use_rviz") == "true",
    )

    # ── Ordered start: controllers after spawn ────────────────
    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                spawn_r1_jsb,
                spawn_r2_jsb,
                spawn_r1_arm,
                spawn_r2_arm,
            ],
        )
    )

    return LaunchDescription([
        # Args
        gui_arg, rviz_arg, verbose_arg,
        # Core
        gazebo,
        rsp,
        spawn_robot,
        controller_manager,
        load_controllers,
        # Avoidance nodes
        collision_monitor,
        r1_ctrl,
        r2_ctrl,
        orchestrator,
        rviz,
    ])