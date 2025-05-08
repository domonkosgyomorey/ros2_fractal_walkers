import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute, SetPen
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Config:
    TARGET_CENTER = (5.5, 5.5)
    TARGET_SIZE = 10.0
    NUM_TURTLES = 3
    START_DEPTH = 2
    SERVICE_TIMEOUT = 2.0
    PEN_MIN_WIDTH = 2
    PEN_MAX_WIDTH = 7
    ANGLE_TOLERANCE = 0.007
    DISTANCE_TOLERANCE = 0.007
    ANGULAR_SPEED_GAIN = 8.0
    LINEAR_SPEED_GAIN = 8.0
    TELEPORT_THETA = 0.0
    SPAWN_X_OFFSET = 0.5
    SPAWN_Y = 5.5
    DEPTH_COLORS = [
        (0, 0, 0),          # Black
        (192, 192, 192),    # Light Gray
        (255, 0, 0),        # Red
        (255, 0, 255),      # Magenta
        (255, 255, 0),      # Yellow
    ]

def hilbert_curve(order, size=1.0):
    points = []
    def hilbert(x, y, xi, xj, yi, yj, n):
        if n <= 0:
            px = x + (xi + yi) / 2
            py = y + (xj + yj) / 2
            points.append((px, py))
        else:
            hilbert(x, y, yi/2, yj/2, xi/2, xj/2, n-1)
            hilbert(x+xi/2, y+xj/2, xi/2, xj/2, yi/2, yj/2, n-1)
            hilbert(x+xi/2+yi/2, y+xj/2+yj/2, xi/2, xj/2, yi/2, yj/2, n-1)
            hilbert(x+xi/2+yi, y+xj/2+yj, -yi/2, -yj/2, -xi/2, -xj/2, n-1)
    hilbert(0.0, 0.0, size, 0.0, 0.0, size, order)
    return points

def normalize_and_center(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = max_x - min_x
    height = max_y - min_y
    scale = Config.TARGET_SIZE / max(width, height)
    normalized_points = []
    for x, y in points:
        nx = (x - min_x) * scale
        ny = (y - min_y) * scale
        cx = nx - (width * scale) / 2 + Config.TARGET_CENTER[0]
        cy = ny - (height * scale) / 2 + Config.TARGET_CENTER[1]
        normalized_points.append((cx, cy))
    return normalized_points

def get_color_by_depth(depth):
    colors = Config.DEPTH_COLORS
    index = (depth - Config.START_DEPTH) % len(colors)
    return colors[index]

def get_pen_width(depth, max_depth):
    if max_depth == 1:
        return Config.PEN_MIN_WIDTH
    return int(Config.PEN_MAX_WIDTH - (depth - Config.START_DEPTH) * (Config.PEN_MAX_WIDTH - Config.PEN_MIN_WIDTH) / (max_depth - 1))

class FractalWalker(Node):
    def __init__(self, turtle_name, depth, max_depth):
        super().__init__(f'fractal_walker_{turtle_name}')
        self.turtle_name = turtle_name
        self.depth = depth
        self.max_depth = max_depth
        raw_points = hilbert_curve(depth)
        self.path = normalize_and_center(raw_points)

        self.target_index = 0
        self.finished = False
        self.pose = None

        self.publisher = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)
        self.cli_teleport = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        self.create_timer(0.01, self.control_loop)

        self.set_pen(True)  # Pen up
        self.teleport(self.path[0])
        self.set_pen(False) # Pen down

    def set_pen(self, pen_off):
        if not self.cli_pen.wait_for_service(timeout_sec=Config.SERVICE_TIMEOUT):
            return
        r, g, b = get_color_by_depth(self.depth)
        width = get_pen_width(self.depth, self.max_depth)
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = pen_off
        future = self.cli_pen.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def teleport(self, pos):
        if not self.cli_teleport.wait_for_service(timeout_sec=Config.SERVICE_TIMEOUT):
            return
        req = TeleportAbsolute.Request()
        req.x = pos[0]
        req.y = pos[1]
        req.theta = Config.TELEPORT_THETA
        future = self.cli_teleport.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.finished or self.pose is None or self.target_index is None:
            return
        if self.target_index >= len(self.path):
            self.finished = True
            self.publisher.publish(Twist())  # stop
            return
        target_x, target_y = self.path[self.target_index]
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        distance = math.sqrt(dx * dx + dy * dy)
        target_theta = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_theta - self.pose.theta), math.cos(target_theta - self.pose.theta))
        twist = Twist()
        if abs(angle_diff) > Config.ANGLE_TOLERANCE:
            twist.angular.z = (1.0 / self.depth) * Config.ANGULAR_SPEED_GAIN * angle_diff
        elif distance > Config.DISTANCE_TOLERANCE:
            twist.linear.x = (1.0 / self.depth) * Config.LINEAR_SPEED_GAIN * distance
        else:
            self.target_index += 1
        self.publisher.publish(twist)

def spawn_turtle_sync(node, name, x, y, theta):
    client = node.create_client(Spawn, '/spawn')
    if not client.wait_for_service(timeout_sec=Config.SERVICE_TIMEOUT):
        return False
    req = Spawn.Request()
    req.name = name
    req.x = x
    req.y = y
    req.theta = theta
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    return future.result() is not None

def main(args=None):
    rclpy.init(args=args)
    node = Node('setup_node')
    num_turtles = Config.NUM_TURTLES
    start_depth = Config.START_DEPTH
    for i in range(2, num_turtles + 1):
        spawn_turtle_sync(node, f'turtle{i}',
                          Config.TARGET_CENTER[0] + i * Config.SPAWN_X_OFFSET,
                          Config.SPAWN_Y,
                          Config.TELEPORT_THETA)
    turtles = []
    for i in range(num_turtles):
        depth = start_depth + i
        turtle_name = f'turtle{i + 1}'
        turtles.append(FractalWalker(turtle_name, depth, start_depth + num_turtles - 1))
    node.destroy_node()
    try:
        while rclpy.ok():
            for t in turtles:
                rclpy.spin_once(t, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    for t in turtles:
        t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
