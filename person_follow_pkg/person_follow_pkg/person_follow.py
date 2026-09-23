""" This node makes the robot follow a person by clustering laser scan
    points into candidate "blobs", picking the one most likely to be a
    person (person-sized width, plausible range, and — once locked on —
    close in angle to where the person was last seen), then driving a
    heading-angle error and a distance error to zero with a proportional
    controller. """
import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

STATE = 'person_following'


def in_forward_fov(i, n, fov_deg):
    angle_from_front = min(i, n - i)
    return angle_from_front <= fov_deg


class PersonFollowNode(Node):
    """ This class wraps the basic functionality of the node """

    def __init__(self):
        super().__init__('person_follow')
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan,
                                 qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # target_angle is heading of neato
        #  target_dist(m) describe where the tracked person currently is, as computed
        # in process_scan.
        self.target_angle = None
        self.target_dist = None
        self.person_found = False
        self.last_target_angle = None   # used for frame-to-frame tracking
        self.last_seen_time = None

        self.declare_parameters(namespace='', parameters=[
            ('Kp_dist', 1),          # gain on distance error
            ('Kp_angle', 2.5),         # gain on heading/angle error
            ('target_distance', 0.5),  # desired standoff distance (m)
            ('max_linear_vel', 0.3),   # m/s cap
            ('max_angular_vel', 1.5),  # rad/s cap
            ('min_range', 0.2),       # ignore returns closer than this (m)
            ('max_range', 1.5),        # ignore returns farther than this (m)
            ('min_person_width', 0.06),  # m, filters out skinny noise
            ('max_person_width', 0.6),  # m, filters out walls/furniture
            ('cluster_range_jump', .2),  # m, max range delta within a cluster
            ('tracking_window_deg', 40.0),  # deg, how far the target can
            # "jump" between frames and
            # still be considered the same
            # person
            ('lost_timeout', 2.0),     # s, stop if not seen this long
            ('gap_threshold', 8.0),  # threshold for gaps in cluster calculation
        ])
        self.Kp_dist = self.get_parameter('Kp_dist').value
        self.Kp_angle = self.get_parameter('Kp_angle').value
        self.target_distance = self.get_parameter('target_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.min_person_width = self.get_parameter('min_person_width').value
        self.max_person_width = self.get_parameter('max_person_width').value
        self.cluster_range_jump = self.get_parameter('cluster_range_jump').value
        self.tracking_window_deg = self.get_parameter('tracking_window_deg').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.gap_threshold = self.get_parameter('gap_threshold').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.active = False
        self.create_subscription(String, 'neato_state', self.process_state, 10)
        self.event_pub = self.create_publisher(String, 'fsm_event', 10)

        print('Person Following!')

    def process_state(self, msg):
        was_active = self.active
        self.active = (msg.data == STATE)
        if was_active and not self.active:
            self.vel_pub.publish(Twist())  # stop once when handing off

    def parameter_callback(self, params):
        """ Allows parameters to be adjusted dynamically, e.g. via
            `ros2 param set`. """
        for param in params:
            if param.type_ == Parameter.Type.DOUBLE and hasattr(self, param.name):
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

    @staticmethod
    def _angle_diff(a, b):
        """Smallest signed difference a-b."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def run_loop(self):
        msg = Twist()
        # print("running")
        lost = True
        if self.person_found and self.last_seen_time is not None:
            age = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
            lost = age > self.lost_timeout

        # always report to the controller, even when not in control
        self.event_pub.publish(String(data='person_lost' if lost else 'person'))
        if not self.active:
            return

        if lost:
            # stop when no one is detected
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            # proprtional control
            angle_error = self.target_angle
            dist_error = self.target_dist - self.target_distance

            angular_z = self.Kp_angle * angle_error
            linear_x = self.Kp_dist * dist_error

            # scales forward speed down (and to zero/negative) as the
            # heading error grows, linear velocity slows
            # linear_x *= max(0.0, math.cos(angle_error))
            # clamps velocity
            angular_z = max(-self.max_angular_vel,
                            min(self.max_angular_vel, angular_z))
            linear_x = max(-self.max_linear_vel,
                           min(self.max_linear_vel, linear_x))

            msg.angular.z = angular_z
            msg.linear.x = linear_x

        # ensure we are not publishing an invalid velocity command
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        """ Cluster the scan into candidate blobs, filter for one that
            looks like a person, and (if tracking) prefer whichever blob
            is closest in angle to where the person was last seen. """
        ranges = msg.ranges
        # print("ranges",ranges)

        n = len(ranges)
        fov_deg = 100
        # screen for valid data
        valid = [(i, r) for i, r in enumerate(ranges)
                 if math.isfinite(r) and self.min_range <= r <= self.max_range
                 and in_forward_fov(i, n, fov_deg)]
        if not valid:
            self.person_found = False
            return

        # group consecutive points with similar ranges into clusters
        clusters = [[valid[0]]]

        for (i, r), (i_prev, r_prev) in zip(valid[1:], valid[:-1]):
            gap = (i - i_prev) % n
            if gap <= self.gap_threshold and abs(r - r_prev) <= self.cluster_range_jump:
                clusters[-1].append((i, r))
            else:
                clusters.append([(i, r)])

        # merge first/last cluster if they're adjacent
        if len(clusters) > 1:
            i_first, r_first = clusters[0][0]
            i_last, r_last = clusters[-1][-1]
            if ((i_first - i_last) % n <= self.gap_threshold
                    and abs(r_first - r_last) <= self.cluster_range_jump):
                clusters[0] = clusters[-1] + clusters[0]
                clusters.pop()
        # Convert into cartesian and filter clusters by width, with clusters witdh
        # non-person-like witdths being filtered out
        candidates = []
        for cluster in clusters:
            pts = [
                (
                    r * math.cos(math.radians(i)),
                    r * math.sin(math.radians(i))
                )
                for i, r in cluster
            ]

            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)

            x0, y0 = pts[0]
            x1, y1 = pts[-1]

            width = math.hypot(x1 - x0, y1 - y0)

            if self.min_person_width <= width <= self.max_person_width:
                candidates.append({
                    'angle': math.atan2(cy, cx),
                    'dist': math.hypot(cx, cy),
                    'width': width
                })
        print('clusters', clusters)
        print('candidates', candidates)
        if not candidates:
            # self.person_found = False
            return

        # Pick candidate. If we're already tracking someone,
        # prefer whoever is closest in angle to the last known position
        # so we can track moving targets otherwise just track the closest cluster.
        best = None
        if self.last_target_angle is not None:
            nearest = min(candidates, key=lambda c: abs(
                self._angle_diff(c['angle'], self.last_target_angle)))
            if abs(self._angle_diff(nearest['angle'], self.last_target_angle)) \
                    <= math.radians(self.tracking_window_deg):
                best = nearest
        else:
            # no prior target — just take the closest valid candidate to bootstrap tracking
            best = min(candidates, key=lambda c: c['dist'])

        if best is None:
            return
        print('best', best)
        self.target_angle = best['angle']
        self.target_dist = best['dist']
        self.last_target_angle = best['angle']
        self.person_found = True
        self.last_seen_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
