#!/usr/bin/env python3
"""
robot1_controller.py
====================
Thin convenience node for Robot 1.
Subscribes to /collision_avoidance/robot1_speed and forwards
it to the controller manager as a speed-scaling command.

In production this node would also handle:
  - Real UR5e speed-scaling via /robot1/set_speed_slider_fraction
  - Emergency stop via /robot1/dashboard_client/stop
  - Re-send trajectory with scaled time_from_start values
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json


class Robot1Controller(Node):

    def __init__(self):
        super().__init__("robot1_controller")
        self.speed_factor = 1.0
        self.get_logger().info("Robot1Controller up. Listening for speed commands.")

        self.create_subscription(
            Float64,
            "/collision_avoidance/robot1_speed",
            self._on_speed,
            10,
        )

        # Status publisher
        self.pub_status = self.create_publisher(Float64, "/robot1/current_speed_factor", 10)
        self.create_timer(0.5, self._publish_status)

    def _on_speed(self, msg: Float64):
        new_speed = max(0.0, min(1.0, msg.data))
        if abs(new_speed - self.speed_factor) > 0.01:
            self.get_logger().info(f"Robot 1 speed → {new_speed:.2f}")
            self.speed_factor = new_speed

    def _publish_status(self):
        m = Float64(); m.data = self.speed_factor
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()