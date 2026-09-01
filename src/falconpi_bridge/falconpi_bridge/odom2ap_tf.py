import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class OdomToApTf(Node):
    def __init__(self):
        super().__init__('odom_to_ap_tf')
        self.pub = self.create_publisher(TFMessage, '/ap/tf', 10)
        self.sub = self.create_subscription(
            Odometry, '/drone_0_visual_slam/odom', self.cb, 10)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        tf_msg = TFMessage()
        tf_msg.transforms = [t]
        self.pub.publish(tf_msg)

def main():
    rclpy.init()
    node = OdomToApTf()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
