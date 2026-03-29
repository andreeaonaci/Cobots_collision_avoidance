#!/usr/bin/env python3
"""
scenario_orchestrator.py
========================
Drives both UR5e robots through a collision-course trajectory.
Listens to /collision_avoidance/robot{1,2}_speed and scales
the trajectory time_from_start on-the-fly so that each robot
moves faster or slower depending on the proximity state.

Flow
────
  1. Wait for controllers to come alive.
  2. Both robots move to HOME pose simultaneously.
  3. Both robots start advancing toward each other (on collision course).
  4. Collision monitor publishes speed factors:
       - dist entering slow_zone → robot2 speed ramps DOWN
       - dist entering danger_zone → robot2 speed = 0.0  (STOPS)
       - robot1 continues at normal speed, passes through
       - dist > resume_zone → robot2 resumes (ramps back up)
  5. Both robots reach their final poses.

Architecture
────────────
  The orchestrator streams small incremental joint-position goals
  to each controller at a fixed rate (50 Hz), scaling the motion
  step size by the current speed factor.  This gives smooth speed
  control without replanning.
"""

import math
import time
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float64
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── UR5e Joint names ──────────────────────────────────────────────
JOINTS_R1 = [
    "robot1_shoulder_pan_joint",
    "robot1_shoulder_lift_joint",
    "robot1_elbow_joint",
    "robot1_wrist_1_joint",
    "robot1_wrist_2_joint",
    "robot1_wrist_3_joint",
]
JOINTS_R2 = [
    "robot2_shoulder_pan_joint",
    "robot2_shoulder_lift_joint",
    "robot2_elbow_joint",
    "robot2_wrist_1_joint",
    "robot2_wrist_2_joint",
    "robot2_wrist_3_joint",
]

# ── Waypoint sequences ────────────────────────────────────────────
# Each row = [pan, lift, elbow, wrist1, wrist2, wrist3]  (radians)
#
# Robot 1 starts at x = -0.70 and extends its end-effector toward +X (table centre).
# Robot 2 starts at x = +0.70 (base rotated 180°), also extends toward table centre.
# Both end-effectors therefore approach each other along the X axis.

WAYPOINTS_R1 = [
    # Home — arms retracted
    [0.0,      -1.5708,  1.5708,  -1.5708,  -1.5708,  0.0],
    # Extend — shoulder out
    [0.0,      -1.2,     1.3,     -1.3,     -1.5708,  0.0],
    # Reach — arm over table
    [0.0,      -0.85,    1.0,     -1.0,     -1.5708,  0.0],
    # Cross — end-effector at table centre  ← collision zone
    [0.0,      -0.55,    0.70,    -0.70,    -1.5708,  0.0],
    # Far — past centre
    [0.0,      -0.30,    0.45,    -0.45,    -1.5708,  0.0],
]

WAYPOINTS_R2 = [
    [0.0,      -1.5708,  1.5708,  -1.5708,  -1.5708,  0.0],
    [0.0,      -1.2,     1.3,     -1.3,     -1.5708,  0.0],
    [0.0,      -0.85,    1.0,     -1.0,     -1.5708,  0.0],
    [0.0,      -0.55,    0.70,    -0.70,    -1.5708,  0.0],
    [0.0,      -0.30,    0.45,    -0.45,    -1.5708,  0.0],
]

SEGMENT_DURATION = 4.0   # seconds per waypoint segment at full speed
CONTROL_RATE     = 50    # Hz — streaming rate for incremental positions


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


def make_point(positions, time_sec):
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in positions]
    secs = int(time_sec)
    nsecs = int((time_sec - secs) * 1e9)
    pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
    return pt


class RobotMover:
    """
    Streams incremental positions to one robot controller.
    Speed is controlled by scaling the time_from_start spacing.
    """

    def __init__(self, node: Node, joints, controller_name):
        self.node       = node
        self.joints     = joints
        self.name       = controller_name
        self.speed      = 1.0      # speed factor 0.0 – 1.0
        self.current    = list(WAYPOINTS_R1[0] if "robot1" in controller_name
                               else WAYPOINTS_R2[0])
        self.waypoints  = (WAYPOINTS_R1 if "robot1" in controller_name
                           else WAYPOINTS_R2)
        self.wp_idx     = 0        # current target waypoint
        self.t          = 0.0      # progress 0→1 in current segment
        self.done       = False

        self.pub = node.create_publisher(
            JointTrajectory,
            f"/{controller_name}/joint_trajectory",
            10,
        )

    def tick(self, dt: float):
        """Advance the robot by dt*speed along its trajectory."""
        if self.done:
            return

        if self.wp_idx >= len(self.waypoints) - 1:
            self.done = True
            return

        start = self.waypoints[self.wp_idx]
        end   = self.waypoints[self.wp_idx + 1]

        step = (dt / SEGMENT_DURATION) * max(self.speed, 0.0)
        self.t += step

        if self.t >= 1.0:
            self.t = 0.0
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints) - 1:
                self.done = True
                self.current = list(self.waypoints[-1])
            else:
                self.current = list(self.waypoints[self.wp_idx])
            return

        self.current = lerp(start, end, self.t)
        self._send(self.current)

    def _send(self, positions):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        traj.points = [make_point(positions, 0.1)]   # 100ms lookahead
        self.pub.publish(traj)


class ScenarioOrchestrator(Node):

    def __init__(self):
        super().__init__("scenario_orchestrator")

        self.declare_parameter("auto_start",       True)
        self.declare_parameter("start_delay_sec",  3.0)

        self.auto_start  = self.get_parameter("auto_start").value
        self.start_delay = self.get_parameter("start_delay_sec").value

        # ── Robot movers ─────────────────────────────────────
        self.r1 = RobotMover(self, JOINTS_R1, "robot1_arm_controller")
        self.r2 = RobotMover(self, JOINTS_R2, "robot2_arm_controller")

        # ── Speed subscriptions ───────────────────────────────
        self.r1_speed = 1.0
        self.r2_speed = 1.0
        self.create_subscription(Float64, "/collision_avoidance/robot1_speed",
                                 lambda m: self._set_speed(1, m.data), 10)
        self.create_subscription(Float64, "/collision_avoidance/robot2_speed",
                                 lambda m: self._set_speed(2, m.data), 10)

        # ── Control timer ────────────────────────────────────
        self.dt      = 1.0 / CONTROL_RATE
        self.running = False
        self.started = False

        if self.auto_start:
            self.start_timer = self.create_timer(self.start_delay, self._auto_start)

        self.ctrl_timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info("ScenarioOrchestrator ready.")

    def _set_speed(self, robot_id, value):
        if robot_id == 1:
            self.r1_speed = value
            self.r1.speed = value
        else:
            self.r2_speed = value
            self.r2.speed = value

    def _auto_start(self):
        if not self.started:
            self.get_logger().info("Auto-starting scenario …")
            self.running = True
            self.started = True
        self.start_timer.cancel()

    def _control_loop(self):
        if not self.running:
            return

        self.r1.tick(self.dt)
        self.r2.tick(self.dt)

        if self.r1.done and self.r2.done:
            self.get_logger().info("Both robots reached final waypoint. Scenario complete.")
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()