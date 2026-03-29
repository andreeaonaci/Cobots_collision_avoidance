#!/usr/bin/env python3
"""
robot2_controller.py
====================
Thin convenience node for Robot 2.
This is the YIELD robot — it stops and resumes based on
the speed commands from collision_monitor.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Robot2Controller(Node):

    def __init__(self):
        super().__init__("robot2_controller")
        self.speed_factor  = 1.0
        self.was_stopped   = False
        self.get_logger().info("Robot2Controller up. This robot YIELDS in conflict.")

        self.create_subscription(
            Float64,
            "/collision_avoidance/robot2_speed",
            self._on_speed,
            10,
        )

        self.pub_status = self.create_publisher(Float64, "/robot2/current_speed_factor", 10)
        self.create_timer(0.5, self._publish_status)

    def _on_speed(self, msg: Float64):
        new_speed = max(0.0, min(1.0, msg.data))
        if abs(new_speed - self.speed_factor) > 0.01:
            if new_speed == 0.0 and not self.was_stopped:
                self.get_logger().warn("Robot 2: STOPPED — yielding to Robot 1.")
                self.was_stopped = True
            elif new_speed > 0.0 and self.was_stopped:
                self.get_logger().info(f"Robot 2: RESUMING at speed factor {new_speed:.2f}")
                self.was_stopped = False
            else:
                self.get_logger().info(f"Robot 2 speed → {new_speed:.2f}")
            self.speed_factor = new_speed

    def _publish_status(self):
        m = Float64(); m.data = self.speed_factor
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()