""" This script commands the neato to move in a square pattern.  """
import time
from time import sleep 
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquareDriver(Node):
    """ This is a message publishing node, sending commands to cmd_vel."""
    def __init__(self):
        """ Initializes the SendMessageNode. No inputs."""
        super().__init__('square_driver')
        # Create a timer that fires ten times per second (10 Hz)
        self.time_start = time.time()
        self.actions = 8
        self.count = 0

        self.move_distance = 1
        self.linear_velocity = 0.25
        self.drive_time = self.move_distance/self.linear_velocity

        self.angular_velocity = 0.2
        self.rotate_angle = math.pi/2
        self.rotate_time = self.rotate_angle /self.angular_velocity

        timer_period = 0.1
        self.create_timer = self.create_timer(timer_period, self.run_square)
        self.pub_vel = self.create_publisher(msg_type=Twist, topic='cmd_vel', qos_profile=10)

    def run_square(self):
        """Moves the neato in the square exactly once. This happens through four rotations and four movements.
        By default the rotations are counter clockwise. It uses self.count and self.action to limit
        the number of possible actions, and action_time to determine actions. When the action timer has
        elapsed, it will always stop temporarily before going to the next action, and it resets the
        action timer for the next action."""
        twist_msg = Twist()
        action_time = time.time() - self.time_start
        if self.count < self.actions:
            if (action_time < self.drive_time) and (self.count % 2 == 0):
                twist_msg.linear.x = self.linear_velocity
                twist_msg.angular.z = 0.0
                self.pub_vel.publish(twist_msg)
            elif (action_time < self.rotate_time) and (self.count % 2 == 1):
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = self.angular_velocity
                self.pub_vel.publish(twist_msg) 
            else:
                self.count += 1
                self.time_start = time.time()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.pub_vel.publish(twist_msg)
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.pub_vel.publish(twist_msg)

def main(args=None):
    """ Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SquareDriver()      # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()