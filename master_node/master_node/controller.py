""" This script is the controller that controls every other nodes """
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FiniteStateController(Node):
    """Controls the Neato through having custom topics, one where it sends data and one
    where it recieves data. It creates neato_state to track the state and
    fsm_event to track the data. """

    def __init__(self):
        super().__init__('finitestatecontroller')
        self.declare_parameter('swap_time', 60.0)
        self.too_close = False
        self.person_seen = False
        self.start_time = self.get_clock().now()

        self.state_pub = self.create_publisher(String, 'neato_state', 10)  # tells others
        self.create_subscription(String, 'fsm_event', self.on_new_information, 10)  # recieves from
        self.create_timer(0.1, self.run_loop)

        print('Finite State Machine!')

    def on_new_information(self, msg):
        """Takes it data from the subscription and updates the logic used to determin the state
        of the Neato."""
        if msg.data == 'close':
            self.too_close = True
        elif msg.data == 'good':
            self.too_close = False
        elif msg.data == 'person':
            self.person_seen = True
        elif msg.data == 'person_lost':
            self.person_seen = False

    def run_loop(self):
        """It will always seek to check if the loop is interrupted by avoiding collisions
        or person following and move to those states. Otherwise, on start, it will seek
        to find a wall and then after 60 seconds, it will swap to square driving. The
        behavior alternates between wall following and square driving every 60 sec."""
        if self.too_close:
            state = 'avoiding_collisions'
        elif self.person_seen:
            state = 'person_following'
        else:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            swap_time = self.get_parameter('swap_time').value
            if int(elapsed // swap_time) % 2 == 0:
                state = 'wall_following'
            else:
                state = 'square_driving'
        self.state_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
