import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan_sub
        self.safe_distance = 5.0 # meters
        self.turning = False

    def scan_callback(self, msg: LaserScan):
        twist = Twist()
        # take the minimum distance in front (±15° around center)
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        front_ranges = []
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if abs(angle) < math.radians(15):  # front sector
                if not math.isinf(ranges[i]):
                    front_ranges.append(ranges[i])

        min_front = min(front_ranges) if front_ranges else float('inf')

        if min_front < self.safe_distance:
            # Obstacle detected → turn left
            twist.angular.z = 0.2
            twist.linear.x = 0.0
            self.get_logger().info(f"Obstacle at {min_front:.2f}m → Turning")
        else:
            # Path clear → go forward
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            self.get_logger().info("Path clear → Moving forward")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()