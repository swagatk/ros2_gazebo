import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AvoidObstacleNode(Node):
    def __init__(self):
        super().__init__('avoid_obstacle_node')

        # Publisher to /demo/cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)

        # Subscriber to /scan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Threshold distance to stop the robot
        self.obstacle_distance_threshold = 0.5  # meters

        self.get_logger().info("AvoidObstacleNode has been started.")

        # Timer for continuously publishing velocity commands
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish at 10 Hz
        self.current_twist = Twist()

    def scan_callback(self, msg):
        # Divide the LaserScan data into three regions: left, front, and right
        ranges = msg.ranges
        num_ranges = len(ranges)
        left = min(ranges[:num_ranges // 3])  # Left third of the scan
        front = min(ranges[num_ranges // 3: 2 * num_ranges // 3])  # Middle third of the scan
        right = min(ranges[2 * num_ranges // 3:])  # Right third of the scan

        self.get_logger().info(f"Sensor readings: {left}, {front}, {right}")
        # Check for obstacles in front
        if left < self.obstacle_distance_threshold or \
                front < self.obstacle_distance_threshold and \
                right > self.obstacle_distance_threshold:
                self.get_logger().warn("Obstacle detected on the left side!, turning right")
                self.turn_right()
        elif right < self.obstacle_distance_threshold or \
                front < self.obstacle_distance_threshold and \
                left > self.obstacle_distance_threshold: 
                self.get_logger().warn("obstacle on right side, turning left")
                self.turn_left()
        elif left < self.obstacle_distance_threshold and \
                front < self.obstacle_distance_threshold and \
                right < self.obstacle_distance_threshold:
            self.get_logger().warn("Obstacle detected in front!, stopping")
            self.turn_right()
        else: # no obstacles detected
            self.move_forward()
    

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0

    def move_forward(self):
        # Publish forward velocity
        self.current_twist.linear.x = 0.7  # Move forward at 0.2 m/s
        self.current_twist.angular.z = 0.0

    def turn_left(self):
        # Turn left by setting a positive angular velocity
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.5  # Turn left at 0.5 rad/s

    def turn_right(self):
        # Turn right by setting a negative angular velocity
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -0.5  # Turn right at 0.5 rad/s


    def publish_velocity(self):
        # Publish the current velocity command
        self.cmd_vel_publisher.publish(self.current_twist)
        self.get_logger().info(f"Publishing velocity: {self.current_twist.linear.x}, {self.current_twist.angular.z}")
        # Add a small delay to avoid flooding the topic
        #self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  


def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()