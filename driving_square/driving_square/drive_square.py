""" This script commands the neato to move in a square pattern.  """
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
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_square)
        self.publish_velocity = self.create_publisher(msg_type=Twist, topic='cmd_vel', qos_profile=10)

    def run_square(self):
        """ Moves it in a square."""
        for _ in range(4):
            # loop to move forward for 1 meter in a square
            self.move_distance(1)
            self.rotate()

    def move_distance(self, distance:int=1):
        """" Moves it in a set distance, with the default being 1"""
        linear_velocity:float = 0.25
        self.drive(linear=linear_velocity)
        sleep(distance/linear_velocity)
        self.drive(linear=0.0, angular=0.0)
        print('Finished moving')

    def drive(self, linear:float=0.25, angular:float=0.0):
        """ Moves it with a set linear velocity"""
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.publish_velocity.publish(twist_msg)

    def rotate(self, angle:float=90): 
        """Rotates the neato with a set angle, given in degrees
        Input: angle (float), an angle to move the Neato by a set degrees"""
        radian_angle:float = angle*math.pi/180
        angular_velocity:float = 0.2
        self.drive(linear=0.0, angular = angular_velocity)      
        sleep(radian_angle/angular_velocity)               
        self.drive(linear=0.0, angular = 0.0)
        print('Finished rotating')
        

def main(args=None):
    """ Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SquareDriver()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()