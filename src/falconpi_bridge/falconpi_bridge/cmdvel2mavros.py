import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

WATCHDOG_TIMEOUT = 0.5
MIN_SETPOINTS_BEFORE_ARM = 20

class CmdVelToMavros(Node):
    def __init__(self):
        super().__init__('cmdvel_to_mavros')

        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, qos_state)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.last_cmd_time = None
        self.setpoint_count = 0
        self.armed = False
        self.current_state = None
        self.mode_switch_in_progress = False

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)

    def state_cb(self, msg: State):
        self.current_state = msg
        if msg.armed and not self.armed:
            self.armed = True
        elif not msg.armed and self.armed:
            self.armed = False

    def publish_velocity(self, vx_enu, vy_enu, vz_enu, yaw_rate_enu):
        """vx/vy/vz in ENU m/s, yaw_rate in rad/s CCW+. No conversion — MAVROS handles ENU->NED internally."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = vx_enu
        msg.twist.linear.y = vy_enu
        msg.twist.linear.z = vz_enu
        msg.twist.angular.z = yaw_rate_enu

        self.pub.publish(msg)
        self.last_cmd_time = time.monotonic()
        self.setpoint_count += 1

        if self.setpoint_count >= MIN_SETPOINTS_BEFORE_ARM and not self.armed and not self.mode_switch_in_progress:
            self.set_mode_and_arm()

    def watchdog_cb(self):
        if self.last_cmd_time is None:
            return
        gap = time.monotonic() - self.last_cmd_time
        if gap > WATCHDOG_TIMEOUT and self.armed:
            self.get_logger().warn(f'No cmd_vel for {gap:.2f}s — triggering disarm')
            self.disarm()

    def set_mode_and_arm(self):
        if self.current_state is None:
            return
        self.mode_switch_in_progress = True
        if self.current_state.mode != 'GUIDED':
            if not self.mode_client.service_is_ready():
                self.mode_switch_in_progress = False
                return
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_client.call_async(req).add_done_callback(self.mode_set_done)
        else:
            self.arm()

    def mode_set_done(self, future):
        try:
            if future.result().mode_sent:
                self.get_logger().info('GUIDED set, arming...')
                self.arm()
            else:
                self.get_logger().error('Failed to set GUIDED')
                self.mode_switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'set_mode failed: {e}')
            self.mode_switch_in_progress = False

    def arm(self):
        if not self.arming_client.service_is_ready():
            self.mode_switch_in_progress = False
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req).add_done_callback(self.arm_done)

    def arm_done(self, future):
        self.mode_switch_in_progress = False
        try:
            if future.result().success:
                self.armed = True
                self.get_logger().info('ARMED')
            else:
                self.get_logger().error('Arming rejected')
        except Exception as e:
            self.get_logger().error(f'arm failed: {e}')

    def disarm(self):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req).add_done_callback(self.disarm_done)

    def disarm_done(self, future):
        try:
            if future.result().success:
                self.armed = False
                self.get_logger().info('DISARMED')
        except Exception as e:
            self.get_logger().error(f'disarm failed: {e}')

def main():
    rclpy.init()
    node = CmdVelToMavros()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
