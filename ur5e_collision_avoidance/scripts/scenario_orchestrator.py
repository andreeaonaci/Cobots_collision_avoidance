#!/usr/bin/env python3
"""
scenario_orchestrator.py — CORRECT OPPOSING MOTION
====================================================
Robot 1 (left,  base yaw=0):    pan=0     → reaches RIGHT toward centre
Robot 2 (right, base yaw=π):    pan=3.14  → also reaches toward centre
                                            (opposite pan because base is flipped)

Visual result:
  R1 ──arm──►        ◄──arm── R2
         they approach each other
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

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

# ── Robot 1: base at x=-0.7, yaw=0, reaches toward +X (centre) ───
# pan=0 means facing +X from its base
WAYPOINTS_R1 = [
    # Home — arm pointing straight up
    [ 0.0,  -1.57,  0.0,  -1.57,  0.0,  0.0],
    # Begin reach — shoulder comes forward
    [ 0.0,  -1.57,  1.0,  -1.57,  0.0,  0.0],
    # Mid reach — arm extending horizontally
    [ 0.0,  -1.2,   1.8,  -2.15,  0.0,  0.0],
    # Full reach — EE at table centre (collision zone)
    [ 0.0,  -0.9,   2.2,  -2.87,  0.0,  0.0],
]

# ── Robot 2: base at x=+0.7, yaw=π, reaches toward -X (centre) ──
# Because the base is rotated 180°:
#   pan = 3.14  →  arm faces -X  (toward Robot 1)
# Lift/elbow/wrist are SAME as R1 (symmetric reach)
WAYPOINTS_R2 = [
    # Home
    [ 3.14,  -1.57,  0.0,  -1.57,  0.0,  0.0],
    # Begin reach
    [ 3.14,  -1.57,  1.0,  -1.57,  0.0,  0.0],
    # Mid reach
    [ 3.14,  -1.2,   1.8,  -2.15,  0.0,  0.0],
    # Full reach toward centre
    [ 3.14,  -0.9,   2.2,  -2.87,  0.0,  0.0],
]

SEGMENT_DURATION = 5.0
CONTROL_RATE     = 50


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


def make_msg(joints, positions, duration_sec=0.12):
    msg = JointTrajectory()
    msg.joint_names = joints
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in positions]
    s = int(duration_sec)
    ns = int((duration_sec - s) * 1e9)
    pt.time_from_start = Duration(sec=s, nanosec=ns)
    msg.points = [pt]
    return msg


class RobotMover:
    def __init__(self, node, joints, topic, waypoints):
        self.node      = node
        self.joints    = joints
        self.waypoints = waypoints
        self.speed     = 1.0
        self.wp_idx    = 0
        self.t         = 0.0
        self.done      = False
        self.pub = node.create_publisher(JointTrajectory, topic, 10)

    def tick(self, dt):
        if self.done or self.wp_idx >= len(self.waypoints) - 1:
            self.done = True
            return
        start = self.waypoints[self.wp_idx]
        end   = self.waypoints[self.wp_idx + 1]
        self.t += (dt / SEGMENT_DURATION) * max(self.speed, 0.0)
        if self.t >= 1.0:
            self.t = 0.0
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints) - 1:
                self.done = True
            return
        self.pub.publish(make_msg(self.joints, lerp(start, end, self.t)))


class ScenarioOrchestrator(Node):

    def __init__(self):
        super().__init__("scenario_orchestrator")
        self.declare_parameter("auto_start",      True)
        self.declare_parameter("start_delay_sec", 3.0)

        delay = self.get_parameter("start_delay_sec").value

        self.r1 = RobotMover(self, JOINTS_R1,
                             "/robot1_arm_controller/joint_trajectory",
                             WAYPOINTS_R1)
        self.r2 = RobotMover(self, JOINTS_R2,
                             "/robot2_arm_controller/joint_trajectory",
                             WAYPOINTS_R2)

        self.create_subscription(Float64, "/collision_avoidance/robot1_speed",
                                 lambda m: setattr(self.r1, 'speed', m.data), 10)
        self.create_subscription(Float64, "/collision_avoidance/robot2_speed",
                                 self._r2_speed, 10)

        self.running = False
        self.dt = 1.0 / CONTROL_RATE
        self.create_timer(delay, self._start)
        self.create_timer(self.dt, self._loop)
        self.get_logger().info("ScenarioOrchestrator ready — robots will face each other.")

    def _r2_speed(self, msg):
        self.r2.speed = msg.data
        if msg.data == 0.0:
            self.get_logger().warn("⚠  Robot 2 STOPPED — yielding to Robot 1")
        elif msg.data > 0.9 and not self.r2.done:
            self.get_logger().info("✔  Robot 2 RESUMED full speed")

    def _start(self):
        self.get_logger().info("▶  Scenario started — robots moving toward each other")
        self.running = True

    def _loop(self):
        if not self.running:
            return
        self.r1.tick(self.dt)
        self.r2.tick(self.dt)
        if self.r1.done and self.r2.done:
            self.get_logger().info("✔  Scenario complete.")
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
