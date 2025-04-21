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
        self.start2 = time.time()
        self.start3 = time.time()
        self.start4 = time.time()
        self.left_flag = False

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
            end = time.time()
            elapsed_time = end - self.start2
            if elapsed_time < 10:
                self.get_logger().warn(f"Obstacle detected on left, front & right: {elapsed_time}")
                self.move_backward()
                self.turn_right()
            else:
                self.start2 = time.time()
        elif left < self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right > self.wall_distance:
            end = time.time()
            elapsed_time = end - self.start3
            if elapsed_time < 10:
                self.get_logger().warn(f"Obstacle detected on front-left! moving right: {elapsed_time}")
                self.turn_right()
            else:
                self.start3 = time.time() # reset the timer
        elif left > self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right < self.wall_distance:
            end = time.time()
            elapsed_time = end - self.start4
            if elapsed_time < 10:
                self.get_logger().warn(f"Obstacle detected on front-right! moving left: {elapsed_time}")
                self.turn_left() 
            else:
                self.start4 = time.time()
        elif left < self.wall_distance and \
                front > self.front_obstacle_threshold and \
                    right > self.wall_distance:
            # Obstacle detected on the left and too far from the wall on the right
            self.get_logger().warn("Obstacle detected on left. Adjust right.")
            self.adjust_right()
        elif  left > self.wall_distance and \
                front > self.front_obstacle_threshold and \
                    right < self.wall_distance:
            # Obstacle detected on the right and too far from the wall on the left
            self.get_logger().warn("Obstacle detected on right. Adjust left.")
            self.adjust_left()
        elif left < self.wall_distance and \
                front > self.front_obstacle_threshold and \
                    right < self.wall_distance:
            # Obstacle detected on left and right. move forward
            self.get_logger().warn("Obstacle detected on left and right. Move forward.")
            self.move_forward()
        elif left > self.wall_distance and \
                front < self.front_obstacle_threshold and \
                    right > self.wall_distance:
            # Obstacle detected in front and too far from the wall on the left
            end = time.time()
            elapsed_time = end - self.start_time
            if elapsed_time < 25:
                if left > right or self.left_flag is True: # more space on left
                    self.get_logger().warn(f"Obstacle detected in front moving left: {elapsed_time}")
                    self.turn_left()
                    self.left_flag = True
                else:
                    self.turn_left()
                    self.left_flag = False
            elif elapsed_time >= 25 and elapsed_time < 50:
                if right > left or self.left_flag is False:
                    self.turn_right()
                    self.get_logger().warn(f"Obstacle detected in front moving right: {elapsed_time}")
                    self.left_flag = False
                else:
                    self.turn_right()
                    self.left_flag = True
            else:
                self.start_time = time.time() # reset the timer
        else:
            # No obstacles detected, move forward
            self.get_logger().info("No obstacles detected. Moving forward.")
            self.move_forward()

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
        self.current_twist.angular.z = -0.5  # Turn left at 0.5 rad/s

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
        self.current_twist.angular.z = 0.5

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