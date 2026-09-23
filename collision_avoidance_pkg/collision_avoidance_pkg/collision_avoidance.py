import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data


class CollisionAvoidanceNode(Node):
    """ Detects close obstacles in front of the robot and steers away
        from it"""

    def __init__(self):
        super().__init__('collision_avoidance')

        self.create_subscription(LaserScan, 'scan', self.process_scan,
                                  qos_profile=qos_profile_sensor_data)
        self.create_timer(0.1, self.run_loop)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.front_dist = None   # closest reading in the front cone (m)
        self.turn_away_sign = 1.0  # which way to turn when triggered;
                                    # flip via param if you prefer a fixed
                                    # escape direction

        self.declare_parameters(namespace='', parameters=[
            ('safety_distance', 0.5),   # trigger threshold (m)
            ('safety_turn_rate', 1.0),  # rad/s while escaping
        ])
        self.safety_distance = self.get_parameter('safety_distance').value
        self.safety_turn_rate = self.get_parameter('safety_turn_rate').value
        self.front_cone_deg = self.get_parameter('front_cone_deg').value

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.type_ == Parameter.Type.DOUBLE and hasattr(self, param.name):
                setattr(self, param.name, param.value)
            elif param.type_ == Parameter.Type.INTEGER and hasattr(self, param.name):
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

    def process_scan(self, msg):

        cone = int(self.front_cone_deg)
        r_frontL = list(msg.ranges[0:cone])
        r_frontR = list(msg.ranges[360 - cone])
        r_front = r_frontL + r_frontR
        # safety check for front
        valid_front = [r for r in r_front if r != 0.0 and math.isfinite(r)]
        if not valid_front:
            self.front_dist = None
        else:
            self.front_dist = min(valid_front)

    def run_loop(self):

        obstacle_close = (self.front_dist is not None
        and self.front_dist < self.safety_distance)
  
        msg = Twist()
        if obstacle_close:
            msg.angular.z = self.turn_away_sign * self.safety_turn_rate
            # proprtional control
            msg.linear.x = max(0.0, 0.1 * (self.front_dist / self.safety_distance))
        else:
            #stop once cleared
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()