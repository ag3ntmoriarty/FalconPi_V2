#!/usr/bin/env python3
"""
Bench velocity test — forward 1 m/s for 1 s on GUIDED mode switch.

Uses ArduPilot's native DDS interface (/ap/* topics/services) via
micro XRCE-DDS agent — NOT MAVROS.

Usage
-----
  ros2 run falconpi_bridge bench_vel_test

Behaviour
---------
* Subscribes to /ap/status and waits.
* The moment the flight mode changes TO GUIDED (mode 4, from any other
  mode), it publishes a 1 m/s forward (ENU +X) velocity setpoint on
  /ap/cmd_vel for exactly 1 second, then publishes a zero-velocity hold
  and stops.
* The script only fires **once** per GUIDED transition — switch away and
  back to GUIDED to trigger it again.

Assumptions
-----------
* Props are OFF — this is a bench test.
* Hardware safety switch is disengaged.
* The drone is already armed and in the air (or at least armed on the
  bench); no takeoff or arming logic is included.
* The micro XRCE-DDS agent is running on the serial port connected to
  the Pixhawk.

Safety
------
This script intentionally does NOT arm, take off, or change modes itself.
It only reacts to an external mode switch so the operator stays in full
control via the RC / GCS.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.msg import Status


# ── ArduCopter mode numbers (from COPTER_MODE enum) ─────────────────
GUIDED_MODE = 4

# ── Tunable constants ────────────────────────────────────────────────
FORWARD_VEL_MPS = 1.0     # m/s in ENU +X (forward)
DURATION_SEC = 1.0         # how long to command the velocity
CMD_RATE_HZ = 20.0         # setpoint publish rate during the burst


class BenchVelTest(Node):
    """Send a short velocity burst when the FC enters GUIDED mode."""

    def __init__(self):
        super().__init__('bench_vel_test')

        # ── State tracking ───────────────────────────────────────────
        self.prev_mode: int | None = None   # last known flight mode
        self.burst_active = False            # True while the 1-s burst is running
        self.burst_timer = None              # timer handle for the burst loop
        self.elapsed = 0.0                   # seconds sent so far

        # ── ArduPilot DDS status subscriber ──────────────────────────
        self.status_sub = self.create_subscription(
            Status, '/ap/status', self._status_cb, 10)

        # ── Velocity command publisher (/ap/cmd_vel) ─────────────────
        self.pub = self.create_publisher(
            TwistStamped, '/ap/cmd_vel', 10)

        self.get_logger().info(
            'bench_vel_test ready — switch to GUIDED (mode 4) to trigger '
            f'{FORWARD_VEL_MPS} m/s forward for {DURATION_SEC} s')

    # ─────────────────────────────────────────────────────────────────
    # Status callback — detect the transition into GUIDED
    # ─────────────────────────────────────────────────────────────────
    def _status_cb(self, msg: Status):
        current_mode = msg.mode

        if (self.prev_mode is not None
                and self.prev_mode != GUIDED_MODE
                and current_mode == GUIDED_MODE
                and not self.burst_active):
            self.get_logger().info(
                f'Mode changed {self.prev_mode} → GUIDED ({GUIDED_MODE}) '
                '— starting velocity burst')
            self._start_burst()

        self.prev_mode = current_mode

    # ─────────────────────────────────────────────────────────────────
    # Velocity burst
    # ─────────────────────────────────────────────────────────────────
    def _start_burst(self):
        self.burst_active = True
        self.elapsed = 0.0
        period = 1.0 / CMD_RATE_HZ
        self.burst_timer = self.create_timer(period, self._burst_tick)

    def _burst_tick(self):
        period = 1.0 / CMD_RATE_HZ

        if self.elapsed >= DURATION_SEC:
            # ── Burst complete — send zero-velocity hold & stop ──────
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('Velocity burst complete — holding zero velocity')
            self.burst_timer.cancel()
            self.burst_active = False
            return

        self._publish_velocity(FORWARD_VEL_MPS, 0.0, 0.0, 0.0)
        self.elapsed += period

    def _publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Publish a velocity command on /ap/cmd_vel (ENU frame).

        ArduPilot DDS /ap/cmd_vel expects TwistStamped in ENU:
          linear.x  = forward (East)
          linear.y  = left    (North)
          linear.z  = up
          angular.z = yaw rate (CCW positive)
        """
        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = 'map'

        tw.twist.linear.x = vx
        tw.twist.linear.y = vy
        tw.twist.linear.z = vz
        tw.twist.angular.z = yaw_rate

        self.pub.publish(tw)


def main():
    rclpy.init()
    node = BenchVelTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
