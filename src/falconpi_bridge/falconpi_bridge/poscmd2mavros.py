import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
import math
import time

WATCHDOG_TIMEOUT = 0.5  
MIN_SETPOINTS_BEFORE_ARM = 20  

class PosCmdToMavros(Node):
    def __init__(self):
        super().__init__('poscmd_to_mavros')

        # MAVROS state topic requires Transient Local durability to connect
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.sub = self.create_subscription(
            PositionCommand, '/drone_0_planning/pos_cmd', self.cb, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, qos_state)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.last_msg_time = None
        self.setpoint_count = 0
        self.armed = False
        self.current_state = None
        
        # Prevents spamming the async service calls once the threshold is crossed
        self.mode_switch_in_progress = False

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_cb)

    def state_cb(self, msg: State):
        self.current_state = msg
        # Sync internal state if armed externally (e.g., via RC)
        if msg.armed and not self.armed:
            self.armed = True
        elif not msg.armed and self.armed:
            self.armed = False

    def cb(self, msg: PositionCommand):
        self.last_msg_time = time.monotonic()
        self.setpoint_count += 1

        pt = PositionTarget()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # ENU to NED mapping
        pt.position.x = msg.position.y
        pt.position.y = msg.position.x
        pt.position.z = -msg.position.z

        pt.velocity.x = msg.velocity.y
        pt.velocity.y = msg.velocity.x
        pt.velocity.z = -msg.velocity.z

        pt.acceleration_or_force.x = msg.acceleration.y
        pt.acceleration_or_force.y = msg.acceleration.x
        pt.acceleration_or_force.z = -msg.acceleration.z

        yaw_ned = (math.pi / 2.0) - msg.yaw
        pt.yaw = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
        pt.yaw_rate = -msg.yaw_dot

        # Explicit Mask: Track Pos, Vel, Accel, Yaw. Ignore Yaw Rate.
        # If ArduPilot rejects this due to acceleration vectors, change to:
        # pt.type_mask = 2496 (Ignores Accel and Yaw Rate)
        pt.type_mask = PositionTarget.IGNORE_YAW_RATE

        self.pub.publish(pt)

        # Trigger sequence once threshold is met, but wait if a switch is already pending
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
        if self.current_state is None:
            self.get_logger().warn('No /mavros/state received yet, waiting...')
            return

        self.mode_switch_in_progress = True

        if self.current_state.mode != 'GUIDED':
            if not self.mode_client.service_is_ready():
                self.get_logger().warn('set_mode service not ready')
                self.mode_switch_in_progress = False
                return
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_client.call_async(req).add_done_callback(self.mode_set_done)
        else:
            self.arm()

    def mode_set_done(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info('GUIDED mode set, proceeding to arm...')
                self.arm()
            else:
                self.get_logger().error('Failed to set GUIDED mode')
                self.mode_switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f'set_mode call failed: {e}')
            self.mode_switch_in_progress = False

    def arm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arming service not ready')
            self.mode_switch_in_progress = False
            return
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req).add_done_callback(self.arm_done)

    def arm_done(self, future):
        self.mode_switch_in_progress = False
        try:
            result = future.result()
            if result.success:
                self.armed = True
                self.get_logger().info('ARMED')
            else:
                self.get_logger().error('Arming rejected by flight controller')
        except Exception as e:
            self.get_logger().error(f'arm call failed: {e}')

    def disarm(self):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arming service not ready for disarm')
            return
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req).add_done_callback(self.disarm_done)

    def disarm_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.armed = False
                self.get_logger().info('DISARMED')
            else:
                self.get_logger().error('Disarm command rejected')
        except Exception as e:
            self.get_logger().error(f'disarm call failed: {e}')

def main():
    rclpy.init()
    node = PosCmdToMavros()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
