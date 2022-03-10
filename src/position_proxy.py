import rospy
from inno_swarm.msg import SpeedPoseStamped  # message for position/speed of a drone
from geometry_msgs.msg import PoseStamped, Point  # message for position/orientation of a drone
from mavros_msgs.srv import SetMode
import numpy as np

from src.utils import numpyfy, pointify, start_daemon_thread

n = 6  # number of drones
RADIUS = 1.0  # radius of safe sphere zone of a drone
DRONE_TAG = 'mavros'


class Drone:
    def __init__(self, name: str):
        # VECTOR: nav_start --- nav_target >
        self.nav_start: PoseStamped = PoseStamped()  # initial (start) point of a drone
        self.nav_target: SpeedPoseStamped = SpeedPoseStamped()  # most last point where drone should move
        self.current_pos: PoseStamped = PoseStamped()  # current drone position
        self.setpoint: PoseStamped = PoseStamped()  # the (medium) position where drone should move (ds)

        self.name = name  # drone name
        self.formation_offset: Point = pointify([0, 0, 0])  # offset from formation center

        rospy.Subscriber(f'/{name}/navigate', SpeedPoseStamped, self.cb)
        rospy.Subscriber(f'/{name}/local_position/pose', PoseStamped, self._set_current)
        self.setpoint_pub = rospy.Publisher(f'/{name}/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.ServiceProxy(f"/{name}/set_mode", SetMode)(custom_mode="OFFBOARD")

        start_daemon_thread(target=self._pos_publishing)
        start_daemon_thread(target=self._t_interpolate)

    def _pos_publishing(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.setpoint.pose:
                self.setpoint_pub.publish(self.setpoint)
            else:
                self.setpoint_pub.publish(self.current_pos)
            r.sleep()

    def _set_current(self, pose: PoseStamped):
        self.current_pos = pose

    def _t_interpolate(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.nav_target.speed:
                self.interpolate()
            r.sleep()

    def interpolate(self):
        start = numpyfy(self.nav_start.pose.position)
        target = numpyfy(self.nav_target.position)
        speed = self.nav_target.speed

        distance = np.linalg.norm(target - start)
        time = distance / speed
        passed = min((rospy.get_time() - self.nav_start.header.stamp.to_sec()) / time, 1.0)  # percents
        print(start, target, passed)

        to = PoseStamped()
        to.header.stamp = rospy.Time.now()
        to.pose.position = pointify(start + (target - start) * passed)
        to.pose.orientation.x = 0.0
        to.pose.orientation.y = 0.0
        to.pose.orientation.z = 0.0
        to.pose.orientation.w = 0.0
        self.setpoint = to

    def cb(self, speed_pose: SpeedPoseStamped):
        if self.setpoint.pose:
            self.nav_start = self.setpoint
        else:
            self.nav_start = self.current_pos
        self.nav_target = speed_pose


rospy.init_node("position_proxy")

drones = [Drone(f'{DRONE_TAG}{i}') for i in range(1, n + 1)]


if __name__ == "__main__":
    rospy.spin()
