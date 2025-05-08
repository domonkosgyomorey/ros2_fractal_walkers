import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def hilbert_curve(order, size=10.0, x_offset=0.5, y_offset=0.5):
    """
    Hilbert-görbe koordináták generálása adott méretű négyzeten belül.
    :param order: A Hilbert-görbe rendje (1–5 javasolt)
    :param size: A teljes kitöltendő tér (pl. 10.0 turtlesimhez)
    :param x_offset, y_offset: A kezdő (bal-alsó) pozíció eltolás
    :return: [(x1, y1), (x2, y2), ...] pontlista
    """
    points = []

    def hilbert(x, y, xi, xj, yi, yj, n):
        if n <= 0:
            px: float = x + (xi + yi) / 2
            py: float = y + (xj + yj) / 2
            points.append((px.__round__(1), py.__round__(1)))
        else:
            hilbert(x, y, yi/2, yj/2, xi/2, xj/2, n-1)
            hilbert(x + xi/2, y + xj/2, xi/2, xj/2, yi/2, yj/2, n-1)
            hilbert(x + xi/2 + yi/2, y + xj/2 + yj/2, xi/2, xj/2, yi/2, yj/2, n-1)
            hilbert(x + xi/2 + yi, y + xj/2 + yj, -yi/2, -yj/2, -xi/2, -xj/2, n-1)

    hilbert(x_offset, y_offset, size, 0.0, 0.0, size, order)
    return points



class FractalWalker(Node):
    def __init__(self):
        super().__init__('fractal_walker')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.path = hilbert_curve(order=4, size=10.0, x_offset=0.5, y_offset=0.5)
        self.target_index = 1
        self.timer = self.create_timer(0.01, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            self.get_logger().info("Waiting for pose...")
            return

        target_x, target_y = self.path[self.target_index]
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        target_theta = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_theta - self.pose.theta)

        self.get_logger().info(f'dx: {dx} dy: {dy}')
        
        twist = Twist()

        if abs(angle_diff) > 0.0001:
            twist.angular.z = 3.0*angle_diff
            twist.linear.x = 0.0
        elif distance > 0.0001:
            twist.linear.x = 4.0*distance
            twist.angular.z = 0.0
        else:
            self.target_index += 1

        self.publisher.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = FractalWalker()
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
