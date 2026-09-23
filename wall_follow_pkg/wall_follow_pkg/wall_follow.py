""" This node makes the robot follow a wall on one side by using two laser
    scan measurements (separated by a known angle) to triangulate both the
    distance to the wall and the angle between the robot's heading and the
    wall.  A proportional controller then drives that angle to zero while
    also correcting the perpendicular distance to a target value. """
import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

STATE = 'wall_following'


class WallFollowNode(Node):
    """ This class wraps the basic functionality of the node """

    def __init__(self):
        super().__init__('wall_follow')
        # subscribes to laser scanner, publishes command velocity
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan,
                                 qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # alpha (wall angle error) and dist (perpendicular wall distance)
        # are computed from the triangulated laser data in process_scan and
        self.alpha = None
        self.dist = None
        self.front_dist = None

        # the following lines set up parameters so behavior can be tuned
        # (and adjusted live, e.g. via `ros2 param set`)
        self.declare_parameters(namespace='', parameters=[
            ('Kp_dist', 1.0),        # gain on distance error
            ('Kp_angle', 1.0),       # gain on heading/angle error
            ('target_distance', 0.6),  # desired distance from the wall (m)
            ('forward_vel', 0.1),    # constant forward speed (m/s)
            ('side', 'right'),       # which side to follow
            ('theta_deg', 60.0),     # angle between the two beams [deg]
            ('safety_distance', 0.6),  # if the front beam sees closer than this, override steering
            ('safety_turn_rate', 2.0),  # angular.z magnitude used while overriding (rad/s)
        ])
        self.Kp_dist = self.get_parameter('Kp_dist').value
        self.Kp_angle = self.get_parameter('Kp_angle').value
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_vel = self.get_parameter('forward_vel').value
        self.side = self.get_parameter('side').value
        self.theta_deg = self.get_parameter('theta_deg').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.safety_turn_rate = self.get_parameter('safety_turn_rate').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.update_beam_indices()

        self.active = False
        self.create_subscription(String, 'neato_state', self.process_state, 10)

        print('Wall Following!')

    def process_state(self, msg):
        was_active = self.active
        self.active = (msg.data == STATE)
        if was_active and not self.active:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.pub_vel.publish(twist_msg)

    def update_beam_indices(self):
        """ Work out which two ranges[] indices to use for triangulation,
            based on which side we're following and the beam separation.

            Index convention: index 0 is straight ahead, and index increases
            counterclockwise (to the robot's LEFT) as angle increases, per
            the LaserScan convention (positive rotation about +Z).

            - "perpendicular" beam points directly out to the chosen side.
            - "forward" beam is offset from the perpendicular beam by
              theta_deg, further towards the front of the robot, so the
              triangle we solve always includes the front-side region
              (helps warn us about walls coming up ahead). """
        theta = int(round(self.theta_deg))
        if self.side == 'left':
            # left is +90 degrees (counterclockwise) from straight ahead
            self.perp_index = 90
            self.fwd_index = 90 - theta   # rotate towards the front (index 0)
        else:
            # right is 270 degrees
            self.perp_index = 270
            self.fwd_index = 270 + theta  # rotate towards the front (wraps via 360->0)
            if self.fwd_index >= 360:
                self.fwd_index -= 360

    def parameter_callback(self, params):
        """ Allows parameters to be adjusted dynamically, e.g. via
            `ros2 param set` or dynamic_reconfigure-style tools. """
        for param in params:
            if param.name == 'Kp_dist' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp_dist = param.value
            elif param.name == 'Kp_angle' and param.type_ == Parameter.Type.DOUBLE:
                self.Kp_angle = param.value
            elif param.name == 'target_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.target_distance = param.value
            elif param.name == 'forward_vel' and param.type_ == Parameter.Type.DOUBLE:
                self.forward_vel = param.value
            elif param.name == 'side' and param.type_ == Parameter.Type.STRING:
                self.side = param.value
                self.update_beam_indices()
            elif param.name == 'theta_deg' and param.type_ == Parameter.Type.DOUBLE:
                self.theta_deg = param.value
                self.update_beam_indices()
            elif param.name == 'safety_distance' and param.type_ == Parameter.Type.DOUBLE:
                self.safety_distance = param.value
            elif param.name == 'safety_turn_rate' and param.type_ == Parameter.Type.DOUBLE:
                self.safety_turn_rate = param.value
        print(self.Kp_dist, self.Kp_angle, self.target_distance,
              self.forward_vel, self.side, self.theta_deg)
        return SetParametersResult(successful=True)

    def run_loop(self):
        if not self.active:
            return
        msg = Twist()
        if self.alpha is None or self.dist is None:
            # haven't triangulated a wall yet; creep forward until we see one
            # print("alpha",self.alpha)
            # print("dist",self.dist)
            msg.linear.x = self.forward_vel
            msg.angular.z = 0.0
        else:
            # forward velocity: could hold constant, or scale down as we
            # approach target_distance head-on; here we just hold constant
            msg.linear.x = self.forward_vel

            # combine the two error terms into one steering command:
            #  - alpha (heading error relative to wall) drives us parallel
            #  - distance error nudges us toward/away from the wall
            dist_error = self.dist - self.target_distance
            print('alpha', self.alpha)
            print('distance', self.dist)
            print('front distance', self.front_dist)
            angular_z = self.Kp_angle * self.alpha + self.Kp_dist * dist_error

            # steering sign convention: positive angular.z turns the robot
            # to the left (CCW). If following the wall on the right, a
            # positive dist_error (too far from wall) should turn us right
            # (negative), so flip sign for right-side following.
            if self.side == 'right':
                angular_z = -angular_z

            msg.angular.z = angular_z
            # collision avoidance
        if self.front_dist is not None and self.front_dist < self.safety_distance:
            # turn away from the wall we're following, hard
            turn_away_sign = 1.0 if self.side == 'right' else -1.0
            msg.angular.z = turn_away_sign * self.safety_turn_rate
            # slow down proportionally to how close we are (never negative)
            msg.linear.x = max(0.0, self.forward_vel
                               * (self.front_dist / self.safety_distance)**5)

        # final safety net: never publish a non-finite velocity command
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.vel_pub.publish(msg)

    def find_valid_range(self, ranges, center_idx, max_search=15):
        n = len(ranges)
        for offset in range(max_search + 1):
            for idx in (center_idx + offset, center_idx - offset):
                idx = idx % n
                r = ranges[idx]
                if r != 0.0 and math.isfinite(r):
                    return idx, r
        return None, None

    def process_scan(self, msg):
        # r_perp = msg.ranges[self.perp_index]
        # r_fwd = msg.ranges[self.fwd_index]
        r_perp = self.find_valid_range(msg.ranges, self.perp_index)
        r_fwd = self.find_valid_range(msg.ranges, self.fwd_index)
        # print(f"len(ranges)={len(msg.ranges)}, perp_index={self.perp_index} -> r_perp={r_perp}, "
        #  f"fwd_index={self.fwd_index} -> r_fwd={r_fwd}")
        r_frontL = msg.ranges[0:45]
        r_frontR = msg.ranges[315:360]
        r_front = r_frontL + r_frontR
        perp_idx, r_perp = self.find_valid_range(msg.ranges, self.perp_index)
        fwd_idx, r_fwd = self.find_valid_range(msg.ranges, self.fwd_index)

        # track the straight-ahead beam separately for the corner/collision
        # safety check, regardless of whether the wall triangulation succeeds
        if min(r_front) == 0.0 or not math.isfinite(min(r_front)):
            self.front_dist = None
        else:
            self.front_dist = min(r_front)

        # checking for 0.0 and non-finite (inf/nan) values ensures both
        # readings are valid.
        if (r_perp == 0.0 or r_fwd == 0.0
                or r_perp is None or r_fwd is None
                or not math.isfinite(r_perp) or not math.isfinite(r_fwd)):
            self.alpha = None
            self.dist = None
            return
        n = len(msg.ranges)
        actual_offset = (fwd_idx - perp_idx) % n
        if actual_offset > n / 2:
            actual_offset -= n  # take the shorter way around
        theta = math.radians(abs(actual_offset))
        # theta = math.radians(self.theta_deg)

        # triangulation: solve for the angle (alpha) between the robot's
        # heading and the wall, and the perpendicular distance to the wall
        #   p1 is vector of the scan perpendicular to the robot and the wall
        #   p2 is the vector of the scan with offset theta and the wall
        #   p1 = (0,r_perp)
        #   p2 = (r_fwd*math.cos(90-theta),r_fwd*math.sin(90-theta))
        #   alpha = p1-p2
        #   dist  = r_perp * cos(alpha)
        alpha = math.atan2(r_fwd * math.cos(theta) - r_perp, r_fwd * math.sin(theta))
        dist = r_perp * math.cos(alpha)

        self.alpha = alpha
        self.dist = dist


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
