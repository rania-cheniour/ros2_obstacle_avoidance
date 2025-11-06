import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('fuzzy_obstacle_avoider')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        self.safe_distance = 1.0  # meters
        self.linear_speed = 0.7
        self.angular_speed = 1.0
        self.laser_ranges = None

        # Extra feature: log status
        self.last_action = "START"

    def get_sector_min(self, ranges, start_deg, end_deg):
        """Get min valid distance in [start_deg, end_deg) (degrees, 0=front, CCW+)"""
        if not ranges:
            return float('inf')
        n = len(ranges)
        angle_inc = 360.0 / n
        start_idx = int(start_deg / angle_inc) % n
        end_idx = int(end_deg / angle_inc) % n

        min_dist = float('inf')
        if start_idx < end_idx:
            sector = ranges[start_idx:end_idx]
        else:
            sector = ranges[start_idx:] + ranges[:end_idx]

        for r in sector:
            if math.isnan(r) or math.isinf(r):
                continue
            if r <= 0.12:  # ignore self-collisions
                continue
            if r < min_dist:
                min_dist = r
        return min_dist if min_dist != float('inf') else 10.0

    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)

        # Define sectors (ROS LIDAR: 0° = back, 180° = front)
        # But TurtleBot3 LIDAR: index 0 = back, mid = front
        # So: front = ±45° around center → (135° to 225° in 0-360 circle where 0=front)
        # To simplify: we treat index[len//2] as FRONT (standard in most setups)
        n = len(msg.ranges)
        front_start = int(0.375 * n)   # 135°
        front_end   = int(0.625 * n)   # 225°
        left_start  = front_end        # 225°
        left_end    = int(0.75 * n)    # 270°
        right_start = int(0.25 * n)    # 90°
        right_end   = front_start      # 135°

        # Helper to get min in index range
        def min_in_range(start, end):
            if start >= end:
                return 10.0
            sector = msg.ranges[start:end]
            valid = [r for r in sector if not (math.isnan(r) or math.isinf(r)) and r > 0.12]
            return min(valid) if valid else 10.0

        min_front = min_in_range(front_start, front_end)
        min_left  = min_in_range(left_start, left_end)
        min_right = min_in_range(right_start, right_end)

        cmd = Twist()

        if min_front < self.safe_distance:
            # Choose side with more space
            if min_left > min_right:
                cmd.angular.z = self.angular_speed  # turn left
                action = "TURNING LEFT"
            else:
                cmd.angular.z = -self.angular_speed # turn right
                action = "TURNING RIGHT"
        else:
            cmd.linear.x = self.linear_speed
            action = "MOVING FORWARD"

        # Log only when action changes
        if action != self.last_action:
            self.get_logger().info(f"✅ {action} | Front: {min_front:.2f}m, Left: {min_left:.2f}m, Right: {min_right:.2f}m")
            self.last_action = action

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())  # stop
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()