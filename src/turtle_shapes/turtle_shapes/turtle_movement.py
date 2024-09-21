import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # Import the Empty service
import time

class TurtleMovement(Node):
    def __init__(self):
        super().__init__('turtle_movement')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Moving turtle in a pentagon')
        self.user_input()

    def user_input(self):
        while True:
            answer = input("Do you want to draw a pentagon (yes/no): ").strip().lower()
            if answer == 'yes':
                self.move_pentagon()
            elif answer == 'no':
                self.get_logger().info('Exiting the program.')
                break
            else:
                self.get_logger().info('Please respond with "yes" or "no".')

    def move_pentagon(self):
        move_msg = Twist()
        turn_msg = Twist()

        move_msg.linear.x = 1.0
        move_msg.angular.z = 0.0
        
        turn_msg.linear.x = 0.0
        turn_msg.angular.z = 1.257  

        forward_duration = 2.0
        turn_duration = 2.0

        for _ in range(5):
            self.publisher_.publish(move_msg)
            self.get_logger().info('Moving forward')
            time.sleep(forward_duration)

            self.publisher_.publish(Twist())  # Stop
            self.get_logger().info('Stop')
            time.sleep(0.5)

            self.publisher_.publish(turn_msg)
            self.get_logger().info('Turning 72 degrees')
            time.sleep(turn_duration)

            self.publisher_.publish(Twist())  # Stop again
            self.get_logger().info('Stopping after turning')
            time.sleep(0.5)

        self.reset_turtle()  # Reset the turtle

    def reset_turtle(self):
        clear_client = self.create_client(Empty, '/clear')
        while not clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting again...')
        
        request = Empty.Request()
        future = clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Drawing cleared.')
        else:
            self.get_logger().error('Failed to call clear service.')

def main(args=None):
    rclpy.init(args=args)
    turtle_movement_node = TurtleMovement()
    rclpy.spin(turtle_movement_node)
    turtle_movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
