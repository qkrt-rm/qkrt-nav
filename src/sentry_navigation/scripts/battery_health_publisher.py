#!/usr/bin/env python3
#
# DUMMY BATTERY PUBLISHER — FOR TESTING ONLY
# ===========================================
# This is a fake battery sensor used to test the mission controller without
# real hardware. It publishes a number that cycles smoothly from 100% down
# to 0% and back up over 40 seconds. This means the mission controller will
# switch between "go to centre" and "go home" roughly every 10 seconds as
# the value crosses the 50% threshold.
#
# WHEN YOU HAVE REAL BATTERY DATA
# --------------------------------
# Delete this file and replace it with a node that reads your actual battery.
# Your replacement node must:
#   - publish to the topic  /battery_health
#   - use message type      std_msgs/Float32
#   - publish a value       between 0.0 (dead) and 100.0 (full)
#
# The mission controller (battery_mission_controller.py) doesn't care where
# the number comes from — just that it arrives on /battery_health.
#
# HOW TO WIRE IT IN
# -----------------
# In navigation.launch.py, replace the battery_health_publisher Node block
# with your real battery reader node. Keep the same topic name.

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


# Total time for one full up-and-down cycle (seconds)
PERIOD_S = 60.0  # 20s going down (100→0), then 20s going back up (0→100)


class BatteryHealthPublisher(Node):
    def __init__(self):
        super().__init__('battery_health_publisher')
        self.pub = self.create_publisher(Float32, '/battery_health', 10)
        self.start_time = self.get_clock().now()
        # Publish once per second so the mission controller gets frequent updates
        self.create_timer(1.0, self._publish)

    def _publish(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        # phase goes 0.0 → 1.0 over one full PERIOD_S cycle, then repeats
        phase = (elapsed % PERIOD_S) / PERIOD_S
        # Triangle wave: starts at 100, hits 0 at the halfway point, returns to 100
        health = 100.0 * (1.0 - 2.0 * abs(phase - 0.5))
        msg = Float32()
        msg.data = float(health)
        self.pub.publish(msg)
        self.get_logger().info(f'Battery health: {health:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryHealthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
