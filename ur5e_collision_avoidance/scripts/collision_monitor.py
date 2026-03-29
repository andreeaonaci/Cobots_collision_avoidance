#!/usr/bin/env python3
"""
collision_monitor.py
====================
Monitors the distance between UR5e end-effectors via TF.
Publishes speed-scale commands and a proximity status topic.

State machine
─────────────
  FREE  ──(dist < slow_zone)──►  SLOWING  ──(dist < danger_zone)──►  STOPPED
  STOPPED  ──(dist > resume_zone)──►  RESUMING  ──(delay)──►  FREE
  SLOWING  ──(dist > slow_zone)──►  FREE

Topics published
────────────────
  /collision_avoidance/status          std_msgs/String  (JSON payload)
  /collision_avoidance/robot1_speed    std_msgs/Float64
  /collision_avoidance/robot2_speed    std_msgs/Float64
  /collision_avoidance/proximity_viz   visualization_msgs/Marker  (line between EEs)
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ── Avoidance state ───────────────────────────────────────────────
class State:
    FREE      = "FREE"
    SLOWING   = "SLOWING"
    STOPPED   = "STOPPED"
    RESUMING  = "RESUMING"


class CollisionMonitor(Node):

    def __init__(self):
        super().__init__("collision_monitor")

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter("monitor_rate_hz",    50.0)
        self.declare_parameter("danger_zone_m",       0.20)
        self.declare_parameter("slow_zone_m",         0.50)
        self.declare_parameter("resume_zone_m",       0.60)
        self.declare_parameter("min_speed_factor",    0.05)
        self.declare_parameter("normal_speed_factor", 1.0)
        self.declare_parameter("yield_robot",         "robot2")
        self.declare_parameter("resume_delay_sec",    0.5)
        self.declare_parameter("log_events",          True)
        self.declare_parameter("log_file",  "/tmp/collision_avoidance_events.json")
        self.declare_parameter("robot1_ee_frame", "robot1_tool0")
        self.declare_parameter("robot2_ee_frame", "robot2_tool0")
        self.declare_parameter("world_frame",     "world")

        self.rate       = self.get_parameter("monitor_rate_hz").value
        self.danger_z   = self.get_parameter("danger_zone_m").value
        self.slow_z     = self.get_parameter("slow_zone_m").value
        self.resume_z   = self.get_parameter("resume_zone_m").value
        self.min_spd    = self.get_parameter("min_speed_factor").value
        self.norm_spd   = self.get_parameter("normal_speed_factor").value
        self.yield_bot  = self.get_parameter("yield_robot").value
        self.resume_dly = self.get_parameter("resume_delay_sec").value
        self.do_log     = self.get_parameter("log_events").value
        self.log_file   = self.get_parameter("log_file").value
        self.r1_ee      = self.get_parameter("robot1_ee_frame").value
        self.r2_ee      = self.get_parameter("robot2_ee_frame").value
        self.world_f    = self.get_parameter("world_frame").value

        # ── State ─────────────────────────────────────────────
        self.state         = State.FREE
        self.stop_time     = None
        self.event_log     = []
        self.last_dist     = 9999.0

        # ── TF listener ───────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_status  = self.create_publisher(String,  "/collision_avoidance/status",       10)
        self.pub_r1_spd  = self.create_publisher(Float64, "/collision_avoidance/robot1_speed", 10)
        self.pub_r2_spd  = self.create_publisher(Float64, "/collision_avoidance/robot2_speed", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/collision_avoidance/markers",  10)
        self.create_subscription(String, "/demo/avoidance_config", self._on_avoidance_config, 10)

        # ── Main timer ────────────────────────────────────────
        period = 1.0 / self.rate
        self.timer = self.create_timer(period, self.monitor_loop)
        self.get_logger().info("CollisionMonitor started. Monitoring at %.0f Hz." % self.rate)

    def _on_avoidance_config(self, msg):
        """Apply live threshold updates from the web dashboard."""
        try:
            cfg = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("Invalid /demo/avoidance_config payload")
            return

        try:
            if "danger_zone_m" in cfg:
                self.danger_z = max(0.05, min(1.5, float(cfg["danger_zone_m"])))
            if "slow_zone_m" in cfg:
                self.slow_z = max(0.10, min(2.0, float(cfg["slow_zone_m"])))
            if self.slow_z <= self.danger_z:
                self.slow_z = self.danger_z + 0.05
            if "resume_zone_m" in cfg:
                self.resume_z = max(self.slow_z + 0.02, min(2.5, float(cfg["resume_zone_m"])))
            else:
                self.resume_z = max(self.resume_z, self.slow_z + 0.02)
            if "yield_robot" in cfg:
                y = str(cfg["yield_robot"]).strip().lower()
                if y in ("robot1", "robot2", "both"):
                    self.yield_bot = y
        except Exception:
            self.get_logger().warn("Failed to apply live avoidance config")

    # ─────────────────────────────────────────────────────────────
    def _get_ee_positions(self):
        """Look up both end-effector positions in world frame via TF."""
        try:
            t1 = self.tf_buffer.lookup_transform(
                self.world_f, self.r1_ee,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            t2 = self.tf_buffer.lookup_transform(
                self.world_f, self.r2_ee,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            p1 = t1.transform.translation
            p2 = t2.transform.translation
            return (p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z)
        except Exception:
            return None, None

    def _distance(self, p1, p2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def _speed_scale_from_distance(self, dist):
        """
        Linear ramp:
          dist >= slow_zone   → normal_speed_factor  (1.0)
          dist <= danger_zone → min_speed_factor      (0.05)
        """
        if dist >= self.slow_z:
            return self.norm_spd
        if dist <= self.danger_z:
            return self.min_spd
        ratio = (dist - self.danger_z) / (self.slow_z - self.danger_z)
        return self.min_spd + ratio * (self.norm_spd - self.min_spd)

    # ─────────────────────────────────────────────────────────────
    def monitor_loop(self):
        p1, p2 = self._get_ee_positions()
        if p1 is None:
            return  # TF not ready yet

        dist = self._distance(p1, p2)
        self.last_dist = dist
        prev_state = self.state

        # ── State transitions ─────────────────────────────────
        if self.state == State.FREE:
            if dist < self.slow_z:
                self.state = State.SLOWING

        elif self.state == State.SLOWING:
            if dist >= self.slow_z:
                self.state = State.FREE
            elif dist < self.danger_z:
                self.state = State.STOPPED
                self.stop_time = self.get_clock().now()

        elif self.state == State.STOPPED:
            if dist > self.resume_z:
                self.state = State.RESUMING
                self.stop_time = self.get_clock().now()

        elif self.state == State.RESUMING:
            elapsed = (self.get_clock().now() - self.stop_time).nanoseconds * 1e-9
            if elapsed >= self.resume_dly:
                self.state = State.FREE

        # ── Log state changes ─────────────────────────────────
        if self.state != prev_state:
            self._log_event(prev_state, self.state, dist, p1, p2)

        # ── Compute speed factors ─────────────────────────────
        r1_spd = self.norm_spd
        r2_spd = self.norm_spd

        if self.state == State.SLOWING:
            scale = self._speed_scale_from_distance(dist)
            if self.yield_bot == "robot2":
                r1_spd = self.norm_spd   # robot 1 keeps going
                r2_spd = scale
            elif self.yield_bot == "both":
                r1_spd = scale
                r2_spd = scale
            else:
                r1_spd = scale
                r2_spd = self.norm_spd

        elif self.state == State.STOPPED:
            if self.yield_bot == "robot2":
                r1_spd = self.norm_spd   # robot 1 keeps going
                r2_spd = 0.0             # robot 2 STOPPED
            elif self.yield_bot == "both":
                r1_spd = 0.0
                r2_spd = 0.0
            else:
                r1_spd = 0.0
                r2_spd = self.norm_spd

        elif self.state == State.RESUMING:
            # Ramp back up gently
            elapsed = (self.get_clock().now() - self.stop_time).nanoseconds * 1e-9
            ramp = min(1.0, elapsed / self.resume_dly)
            if self.yield_bot == "robot2":
                r2_spd = self.min_spd + ramp * (self.norm_spd - self.min_spd)
            elif self.yield_bot == "both":
                r1_spd = self.min_spd + ramp * (self.norm_spd - self.min_spd)
                r2_spd = self.min_spd + ramp * (self.norm_spd - self.min_spd)
            else:
                r1_spd = self.min_spd + ramp * (self.norm_spd - self.min_spd)

        # ── Publish speed factors ─────────────────────────────
        r1_msg = Float64(); r1_msg.data = float(r1_spd)
        r2_msg = Float64(); r2_msg.data = float(r2_spd)
        self.pub_r1_spd.publish(r1_msg)
        self.pub_r2_spd.publish(r2_msg)

        # ── Publish status JSON ───────────────────────────────
        status = {
            "state":       self.state,
            "distance_m":  round(dist, 4),
            "robot1_speed": round(r1_spd, 3),
            "robot2_speed": round(r2_spd, 3),
            "slow_zone_m": round(self.slow_z, 3),
            "danger_zone_m": round(self.danger_z, 3),
            "resume_zone_m": round(self.resume_z, 3),
            "yield_robot": self.yield_bot,
            "stamp":       self.get_clock().now().nanoseconds,
        }
        smsg = String(); smsg.data = json.dumps(status)
        self.pub_status.publish(smsg)

        # ── Publish visual markers ────────────────────────────
        self._publish_markers(p1, p2, dist, r1_spd, r2_spd)

    # ─────────────────────────────────────────────────────────────
    def _publish_markers(self, p1, p2, dist, r1_spd, r2_spd):
        ma = MarkerArray()

        # Line between end-effectors
        line = Marker()
        line.header.frame_id = self.world_f
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "ee_connection"; line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.01

        if dist < self.danger_z:
            line.color.r, line.color.g, line.color.b = 1.0, 0.0, 0.0
        elif dist < self.slow_z:
            line.color.r, line.color.g, line.color.b = 1.0, 0.65, 0.0
        else:
            line.color.r, line.color.g, line.color.b = 0.0, 1.0, 0.0
        line.color.a = 0.9

        from geometry_msgs.msg import Point
        pt1 = Point(); pt1.x, pt1.y, pt1.z = p1
        pt2 = Point(); pt2.x, pt2.y, pt2.z = p2
        line.points = [pt1, pt2]
        line.lifetime = Duration(sec=0, nanosec=int(0.1e9))
        ma.markers.append(line)

        # Sphere at each EE
        for idx, (pos, spd) in enumerate([(p1, r1_spd), (p2, r2_spd)]):
            sp = Marker()
            sp.header.frame_id = self.world_f
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.ns = "ee_spheres"; sp.id = idx + 1
            sp.type = Marker.SPHERE
            sp.action = Marker.ADD
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = pos
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = 0.06
            sp.color.a = 0.8
            if spd == 0.0:
                sp.color.r, sp.color.g, sp.color.b = 1.0, 0.0, 0.0   # red = stopped
            elif spd < 0.5:
                sp.color.r, sp.color.g, sp.color.b = 1.0, 0.65, 0.0  # amber = slow
            else:
                sp.color.r, sp.color.g, sp.color.b = 0.2, 0.8, 0.3   # green = normal
            sp.lifetime = Duration(sec=0, nanosec=int(0.1e9))
            ma.markers.append(sp)

        # ── Text: distance label floating above the midpoint ─────
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2
        mid_z = (p1[2] + p2[2]) / 2 + 0.12
        txt = Marker()
        txt.header.frame_id = self.world_f
        txt.header.stamp = self.get_clock().now().to_msg()
        txt.ns = "ee_distance_text"; txt.id = 10
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = mid_x
        txt.pose.position.y = mid_y
        txt.pose.position.z = mid_z
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.06
        if dist < self.danger_z:
            txt.color.r, txt.color.g, txt.color.b = 1.0, 0.2, 0.2
        elif dist < self.slow_z:
            txt.color.r, txt.color.g, txt.color.b = 1.0, 0.7, 0.0
        else:
            txt.color.r, txt.color.g, txt.color.b = 0.3, 1.0, 0.4
        txt.color.a = 1.0
        txt.text = f"{dist:.3f} m"
        txt.lifetime = Duration(sec=0, nanosec=int(0.15e9))
        ma.markers.append(txt)

        # ── Text: state label just above the distance label ───────
        state_col = {
            "FREE":     (0.3, 1.0, 0.4),
            "SLOWING":  (1.0, 0.7, 0.0),
            "STOPPED":  (1.0, 0.2, 0.2),
            "RESUMING": (0.3, 0.6, 1.0),
        }.get(self.state, (0.7, 0.7, 0.7))
        stxt = Marker()
        stxt.header.frame_id = self.world_f
        stxt.header.stamp = self.get_clock().now().to_msg()
        stxt.ns = "state_text"; stxt.id = 20
        stxt.type = Marker.TEXT_VIEW_FACING
        stxt.action = Marker.ADD
        stxt.pose.position.x = mid_x
        stxt.pose.position.y = mid_y
        stxt.pose.position.z = mid_z + 0.10
        stxt.pose.orientation.w = 1.0
        stxt.scale.z = 0.07
        stxt.color.r, stxt.color.g, stxt.color.b = state_col
        stxt.color.a = 1.0
        stxt.text = self.state
        stxt.lifetime = Duration(sec=0, nanosec=int(0.15e9))
        ma.markers.append(stxt)

        self.pub_markers.publish(ma)

    # ─────────────────────────────────────────────────────────────
    def _log_event(self, from_state, to_state, dist, p1, p2):
        entry = {
            "time_sec": self.get_clock().now().nanoseconds * 1e-9,
            "transition": f"{from_state} → {to_state}",
            "distance_m": round(dist, 4),
            "robot1_ee": [round(v, 4) for v in p1],
            "robot2_ee": [round(v, 4) for v in p2],
        }
        self.event_log.append(entry)
        self.get_logger().info(
            f"[STATE] {from_state} → {to_state}  |  dist={dist:.4f} m"
        )
        if self.do_log:
            try:
                with open(self.log_file, "w") as f:
                    json.dump(self.event_log, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f"Log write failed: {e}")


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()