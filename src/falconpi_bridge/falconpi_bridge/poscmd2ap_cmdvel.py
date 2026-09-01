import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from std_srvs.srv import Trigger
import time

WATCHDOG_TIMEOUT = 0.5
MIN_SETPOINTS_BEFORE_ARM = 20
GUIDED_MODE = 4  # ArduCopter GUIDED

class PosCmdToApCmdVel(Node):
    def __init__(self):
        super().__init__('poscmd_to_ap_cmdvel')

        self.pub = self.create_publisher(
            TwistStamped, '/ap/cmd_vel', 10)
        self.sub = self.create_subscription(
            PositionCommand, '/drone_0_planning/pos_cmd', self.cb, 10)

        self.arming_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.prearm_client = self.create_client(Trigger, '/ap/prearm_check')

        self.last_msg_time = None
        self.setpoint_count = 0
        self.armed = False
        self.mode_switch_in_progress = False

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)

    def cb(self, msg: PositionCommand):
        self.last_msg_time = time.monotonic()
        self.setpoint_count += 1

        tw = TwistStamped()
        tw.header.stamp = self.get_clock().now().to_msg()
        tw.header.frame_id = 'map'

        # No frame conversion — /ap/cmd_vel is ENU, same as pos_cmd
        tw.twist.linear.x = msg.velocity.x
        tw.twist.linear.y = msg.velocity.y
        tw.twist.linear.z = msg.velocity.z
        tw.twist.angular.z = msg.yaw_dot

        self.pub.publish(tw)

        if self.setpoint_count >= MIN_SETPOINTS_BEFORE_ARM and not self.armed and not self.mode_switch_in_progress:
            self.set_mode_and_arm()

    def watchdog_cb(self):
        if self.last_msg_time is None:
            return
        gap = time.monotonic() - self.last_msg_time
        if gap > WATCHDOG_TIMEOUT and self.armed:
            self.get_logger().warn(f'No pos_cmd for {gap:.2f}s — triggering disarm')
            self.disarm()

    def set_mode_and_arm(self):
        self.mode_switch_in_progress = True
        if not self.mode_client.service_is_ready():
            self.get_logger().warn('mode_switch service not ready')
            self.mode_switch_in_progress = False
            return
        req = ModeSwitch.Request()
        req.mode = GUIDED_MODE
        self.mode_client.call_async(req).add_done_callback(self.mode_set_done)

    def mode_set_done(self, future):
        try:
            result = future.result()
            if result.status:
                self.get_logger().info(f'Mode set (curr_mode={result.curr_mode}), checking prearm...')
                self.check_prearm_and_arm()
            else:
                self.get_logger().error('Failed to set GUIDED mode')
                self.mode_switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'mode_switch call failed: {e}')
            self.mode_switch_in_progress = False

    def check_prearm_and_arm(self):
        if not self.prearm_client.service_is_ready():
            self.get_logger().warn('prearm_check service not ready')
            self.mode_switch_in_progress = False
            return
        self.prearm_client.call_async(Trigger.Request()).add_done_callback(self.prearm_done)

    def prearm_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.arm()
            else:
                self.get_logger().error(f'Prearm check failed: {result.message}')
                self.mode_switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'prearm_check call failed: {e}')
            self.mode_switch_in_progress = False

    def arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arm_motors service not ready')
            self.mode_switch_in_progress = False
            return
        req = ArmMotors.Request()
        req.arm = True
        self.arming_client.call_async(req).add_done_callback(self.arm_done)

    def arm_done(self, future):
        self.mode_switch_in_progress = False
        try:
            result = future.result()
            if result.result:
                self.armed = True
                self.get_logger().info('ARMED')
            else:
                self.get_logger().error('Arming rejected by flight controller')
        except Exception as e:
            self.get_logger().error(f'arm_motors call failed: {e}')

    def disarm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arm_motors service not ready for disarm')
            return
        req = ArmMotors.Request()
        req.arm = False
        self.arming_client.call_async(req).add_done_callback(self.disarm_done)

    def disarm_done(self, future):
        try:
            result = future.result()
            if result.result:
                self.armed = False
                self.get_logger().info('DISARMED')
            else:
                self.get_logger().error('Disarm command rejected')
        except Exception as e:
            self.get_logger().error(f'disarm call failed: {e}')

def main():
    rclpy.init()
    node = PosCmdToApCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
