import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Moving turtle in a square')
        self.move_square()

    def move_square(self):
        move_msg = Twist()
        turn_msg = Twist()

        # Move straight parameters
        move_msg.linear.x = 1.0  # Forward speed
        move_msg.angular.z = 0.0  # No rotation
        
        # Turn parameters
        turn_msg.linear.x = 0.0  # No forward motion
        turn_msg.angular.z = 0.785 # Rotation speed for turning (adjust as needed)

        # Define time intervals (seconds)
        forward_duration = 2.0  # Time to move forward (adjust as needed)
        turn_duration = 1.5  # Time to turn 90 degrees (adjust as needed)

        for _ in range(5):  # Repeat 4 times to form a square
            # Move forward
            self.publisher_.publish(move_msg)
            self.get_logger().info('Moving forward')
            time.sleep(forward_duration)

            # Stop after moving forward
            self.publisher_.publish(Twist())  # Stop the turtle
            self.get_logger().info('Stopping after moving forward')
            time.sleep(0.5)  # Small pause to ensure stop

            # Turn 90 degrees
            self.publisher_.publish(turn_msg)
            self.get_logger().info('Turning 90 degrees')
            time.sleep(turn_duration)

            # Stop after turning
            self.publisher_.publish(Twist())  # Stop the turtle again
            self.get_logger().info('Stopping after turning')
            time.sleep(0.5)  # Small pause to ensure stop before moving again

def main(args=None):
    rclpy.init(args=args)
    move_square_node = MoveSquare()
    rclpy.spin(move_square_node)
    move_square_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






