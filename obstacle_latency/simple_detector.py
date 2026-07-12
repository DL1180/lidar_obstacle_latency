import math
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Detector(Node):

    def __init__(self):
        super().__init__('simple_detector')

        self.obstacle_present = False

        self.create_subscription(
            LaserScan,
            '/scan',
            self.check_scan,
            qos_profile_sensor_data
        )

        print('Waiting for LiDAR data...')

    def check_scan(self, scan):
        obstacle = False
        closest_distance = float('inf')

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            # Check 15 degrees to the left and right of the front.
            if -math.radians(15) <= angle <= math.radians(15):
                if math.isfinite(distance) and distance < 0.50:
                    obstacle = True
                    closest_distance = min(closest_distance, distance)

        # Print only when a new obstacle appears.
        if obstacle and not self.obstacle_present:
            current_time = self.get_clock().now().nanoseconds

            scan_time = (
                scan.header.stamp.sec * 1_000_000_000
                + scan.header.stamp.nanosec
            )

            latency_ms = (current_time - scan_time) / 1_000_000

            print()
            print('OBSTACLE DETECTED')
            print(f'Distance: {closest_distance:.3f} m')
            print(f'Scan-to-detection latency: {latency_ms:.3f} ms')

        if not obstacle and self.obstacle_present:
            print('OBSTACLE CLEARED')

        self.obstacle_present = obstacle


def main():
    rclpy.init()
    node = Detector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
