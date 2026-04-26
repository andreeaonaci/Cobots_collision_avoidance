#!/usr/bin/env python3
"""
scenario_orchestrator.py
========================
Runs coordinated trajectories for both UR5e robots.

Important: Robot 2 uses the same pan orientation as Robot 1 so the
tool/sensor orientation remains consistent before and after Start.
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
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

# ── Robot 2: base is pre-rotated 180° in URDF, so pan=0 faces Robot 1 ──
WAYPOINTS_R2 = [
    # Home
    [ 0.0,  -1.57,  0.0,  -1.57,  0.0,  0.0],
    # Begin reach
    [ 0.0,  -1.57,  1.0,  -1.57,  0.0,  0.0],
    # Mid reach
    [ 0.0,  -1.2,   1.8,  -2.15,  0.0,  0.0],
    # Full reach toward centre
    [ 0.0,  -0.9,   2.2,  -2.87,  0.0,  0.0],
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
        self.auto_speed = 1.0
        self.manual_scale = 1.0
        self.wp_idx    = 0
        self.t         = 0.0
        self.done      = False
        self.pub = node.create_publisher(JointTrajectory, topic, 10)

    def tick(self, dt, segment_duration):
        if self.done or self.wp_idx >= len(self.waypoints) - 1:
            self.done = True
            return
        start = self.waypoints[self.wp_idx]
        end   = self.waypoints[self.wp_idx + 1]
        speed = max(self.auto_speed * self.manual_scale, 0.0)
        self.t += (dt / max(segment_duration, 0.1)) * speed
        if self.t >= 1.0:
            self.t = 0.0
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints) - 1:
                self.done = True
            return
        self.pub.publish(make_msg(self.joints, lerp(start, end, self.t)))

    def publish_home(self):
        self.pub.publish(make_msg(self.joints, self.waypoints[0], duration_sec=1.0))


class ScenarioOrchestrator(Node):

    def __init__(self):
        super().__init__("scenario_orchestrator")
        self.declare_parameter("auto_start",      False)
        self.declare_parameter("start_delay_sec", 3.0)
        self.declare_parameter("segment_duration_sec", SEGMENT_DURATION)

        delay = self.get_parameter("start_delay_sec").value
        self.segment_duration = float(self.get_parameter("segment_duration_sec").value)

        self.r1 = RobotMover(self, JOINTS_R1,
                             "/robot1_arm_controller/joint_trajectory",
                             WAYPOINTS_R1)
        self.r2 = RobotMover(self, JOINTS_R2,
                             "/robot2_arm_controller/joint_trajectory",
                             WAYPOINTS_R2)

        self.create_subscription(Float64, "/collision_avoidance/robot1_speed",
                                 self._r1_speed, 10)
        self.create_subscription(Float64, "/collision_avoidance/robot2_speed",
                                 self._r2_speed, 10)
        self.create_subscription(String, "/demo/control", self._control_cmd, 10)
        self.create_subscription(String, "/demo/motion_config", self._motion_cfg, 10)

        self.pub_state = self.create_publisher(String, "/demo/scenario_state", 10)
        self.create_timer(0.2, self._publish_state)

        self.running = False
        self.paused = False
        self._was_stopped = False   # track r2 stop so RESUMED only logs once
        self._done_logged = False
        self.dt = 1.0 / CONTROL_RATE

        auto_start = bool(self.get_parameter("auto_start").value)
        self._start_timer = None
        if auto_start:
            self._start_timer = self.create_timer(delay, self._start)
            self.get_logger().info("Auto-start enabled.")
        else:
            self._publish_home_pose()
            self.get_logger().info("Auto-start disabled. Waiting for /demo/control start.")

        self.create_timer(self.dt, self._loop)
        self.get_logger().info("ScenarioOrchestrator ready — web UI can start/reset the demo.")

    def _reset_movers(self):
        """Reset both movers so the scenario can run again cleanly."""
        self.r1.wp_idx = 0; self.r1.t = 0.0; self.r1.done = False
        self.r2.wp_idx = 0; self.r2.t = 0.0; self.r2.done = False
        self.r1.auto_speed = 1.0; self.r2.auto_speed = 1.0
        self.r1.manual_scale = 1.0; self.r2.manual_scale = 1.0
        self._was_stopped = False
        self._done_logged = False

    def _publish_home_pose(self):
        self.r1.publish_home()
        self.r2.publish_home()

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            "running": self.running,
            "paused": self.paused,
            "done": self.r1.done and self.r2.done,
            "segment_duration": round(self.segment_duration, 3),
            "r1_manual_scale": round(self.r1.manual_scale, 3),
            "r2_manual_scale": round(self.r2.manual_scale, 3),
        })
        self.pub_state.publish(msg)

    def _r1_speed(self, msg):
        self.r1.auto_speed = max(0.0, min(1.5, msg.data))

    def _r2_speed(self, msg):
        self.r2.auto_speed = max(0.0, min(1.5, msg.data))
        if msg.data == 0.0 and self.running and not self._was_stopped:
            self._was_stopped = True
            self.get_logger().warn("⚠  Robot 2 STOPPED — yielding to Robot 1")
        elif msg.data > 0.9 and self._was_stopped and self.running:
            self._was_stopped = False
            self.get_logger().info("✔  Robot 2 RESUMED full speed")

    def _control_cmd(self, msg):
        try:
            payload = json.loads(msg.data)
            action = str(payload.get("action", "")).lower().strip()
        except Exception:
            self.get_logger().warn("Invalid /demo/control payload")
            return

        if action == "start":
            self._reset_movers()
            self.running = True
            self.paused = False
            self.get_logger().info("▶  Scenario started from web UI")
        elif action == "pause":
            if self.running:
                self.paused = True
                self.get_logger().info("⏸  Scenario paused from web UI")
        elif action == "resume":
            if self.running:
                self.paused = False
                self.get_logger().info("▶  Scenario resumed from web UI")
        elif action == "reset":
            self.running = False
            self.paused = False
            self._reset_movers()
            self._publish_home_pose()
            self.get_logger().info("↺  Scenario reset from web UI")
        else:
            self.get_logger().warn(f"Unknown action on /demo/control: {action}")

    def _motion_cfg(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Invalid /demo/motion_config payload")
            return

        if "segment_duration" in payload:
            try:
                self.segment_duration = max(1.0, min(15.0, float(payload["segment_duration"])))
            except Exception:
                pass
        if "r1_manual_scale" in payload:
            try:
                self.r1.manual_scale = max(0.0, min(1.5, float(payload["r1_manual_scale"])))
            except Exception:
                pass
        if "r2_manual_scale" in payload:
            try:
                self.r2.manual_scale = max(0.0, min(1.5, float(payload["r2_manual_scale"])))
            except Exception:
                pass

    def _start(self):
        # One-shot: cancel immediately so this does not repeat
        if self._start_timer is not None:
            self._start_timer.cancel()
        self._reset_movers()
        self.get_logger().info("▶  Scenario started — robots moving toward each other")
        self.running = True
        self.paused = False

    def _loop(self):
        if not self.running or self.paused:
            return
        self.r1.tick(self.dt, self.segment_duration)
        self.r2.tick(self.dt, self.segment_duration)
        if self.r1.done and self.r2.done:
            if not self._done_logged:
                self.get_logger().info("✔  Scenario complete — collision avoidance demo finished.")
                self._done_logged = True
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
