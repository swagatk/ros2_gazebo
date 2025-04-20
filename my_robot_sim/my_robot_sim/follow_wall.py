import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')

        # Publisher to /demo/cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)

        # Subscriber to /scan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Desired distance from the wall
        self.wall_distance = 0.5  # meters

        # Threshold distance to detect obstacles in front
        self.front_obstacle_threshold = 1.0  # meters

        # Timer for continuously publishing velocity commands
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publish at 10 Hz
        self.current_twist = Twist()

        self.get_logger().info("WallFollowingNode has been started.")

        self.start_time = time.time()

    def scan_callback(self, msg):
        # Divide the LaserScan data into three regions: left, front, and right
        ranges = msg.ranges
        num_ranges = len(ranges)

        left = min(ranges[:num_ranges // 3])  # Left third of the scan
        front = min(ranges[num_ranges // 3: 2 * num_ranges // 3])  # Middle third of the scan
        right = min(ranges[2 * num_ranges // 3:])  # Right third of the scan

        self.get_logger().info(f"Sensor readings: Left={left}, Front={front}, Right={right}")

        if left < self.wall_distance and \
                front < self.front_obstacle_threshold and \
                right < self.wall_distance:
                self.get_logger().warn("Obstacle detected in front and too close to the wall! moving backward.")
                self.move_backward()
        elif left < self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right > self.wall_distance:
            # Obstacle detected in front and too close to the wall on the left
            self.get_logger().warn("Obstacle detected in front and too close to the wall on the left! Adjust right.")
            self.adjust_right()
        elif left > self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right < self.wall_distance:
            # Obstacle detected in front and too close to the wall on the right
            self.get_logger().warn("Obstacle detected in front and too close to the wall on the right! Adjust left.")
            self.adjust_left()
        elif left > self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right > self.wall_distance:
            # Obstacle detected in front and too far from the wall on the left
            end = time.time()
            elapsed_time = end - self.start_time
            if elapsed_time < 50:
                self.turn_left()
                self.get_logger().warn(f"Obstacle detected in front moving left: {elapsed_time}")
            elif elapsed_time >= 50 and elapsed_time < 100:
                self.turn_right()
                self.get_logger().warn(f"Obstacle detected in front moving right: {elapsed_time}")
            else:
                self.start_time = time.time() # reset the timer

        else:
            # No obstacles detected, move forward
            self.get_logger().info("No obstacles detected. Moving forward.")
            self.move_forward()

#      # Wall-following logic
#        if front < self.front_obstacle_threshold:
#            # Obstacle detected in front, turn left
#            self.get_logger().warn("Obstacle detected in front! Turning left.")
#            self.turn_left()
#        elif left < self.wall_distance:
#            # Too close to the wall on the left, turn right slightly
#            self.get_logger().info("Too close to the wall on the left. Adjusting right.")
#            self.adjust_right()
#        elif right < self.wall_distance:
#            # Too close to the wall on the right, turn left slightly
#            self.get_logger().info("Too close to the wall on the right. Adjusting left.")
#            self.adjust_left()
#        elif left > self.wall_distance and left < self.front_obstacle_threshold:
#            # Too far from the wall on the left, turn left slightly
#            self.get_logger().info("Too far from the wall on the left. Adjusting left.")
#            self.adjust_left()
#        elif right > self.wall_distance and right < self.front_obstacle_threshold:
#            # Too far from the wall on the right, turn right slightly
#            self.get_logger().info("Too far from the wall on the right. Adjusting right.")
#            self.adjust_right()
#        else:
#            # Maintain distance and move forward
#            self.get_logger().info("No wall detected. Moving forward.")
#            self.move_forward()

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0

    def move_forward(self):
        # Move forward at a constant speed
        self.current_twist.linear.x = 0.5  # Forward speed
        self.current_twist.angular.z = 0.0

    def turn_left(self):
        # Turn left by setting a positive angular velocity
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.5  # Turn left at 0.5 rad/s

    def adjust_left(self):
        # Slightly adjust left to maintain distance from the wall
        self.current_twist.linear.x = 0.1
        self.current_twist.angular.z = 0.2  # Small left turn

    def adjust_right(self):
        # Slightly adjust right to maintain distance from the wall
        self.current_twist.linear.x = 0.1
        self.current_twist.angular.z = -0.2  # Small right turn

    def move_backward(self):
        # Move backward at a constant speed
        self.current_twist.linear.x = -0.5
        self.current_twist.angular.z = 0.0
    def turn_right(self):
        # Turn right by setting a negative angular velocity
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -0.5

    def publish_velocity(self):
        # Publish the current velocity command
        self.cmd_vel_publisher.publish(self.current_twist)
        #self.get_logger().info(f"Publishing velocity: Linear={self.current_twist.linear.x}, Angular={self.current_twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()