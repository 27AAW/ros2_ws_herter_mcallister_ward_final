
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# Replace tf_transformations with custom function
def euler_from_quaternion(quat):
    x, y, z, w = quat
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Controller node starting...")

        # Publisher for /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publisher to /cmd_vel initialized")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF listener initialized")

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Control loop timer started")

        # State machine variables
        self.state = 'FORWARD'
        self.start_pose = None

    def control_loop(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            q = trans.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"Current pose: x={trans.transform.translation.x:.2f}, "
                                   f"y={trans.transform.translation.y:.2f}, yaw={yaw:.2f}")

            twist = Twist()
            if self.state == 'FORWARD':
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            elif self.state == 'TURN':
                twist.linear.x = 0.0
                twist.angular.z = 0.5

            self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
