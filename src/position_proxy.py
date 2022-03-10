import math
from typing import List

import rospy
from inno_swarm.msg import SpeedPoseStamped  # message for position/speed of a drone
from geometry_msgs.msg import PoseStamped, Point  # message for position/orientation of a drone
from mavros_msgs.srv import SetMode
import numpy as np

from src.utils import numpyfy, pointify, start_daemon_thread, rotate_vect

n = 2  # number of drones
RADIUS = 1.2  # radius of safe sphere zone of a drone
DRONE_TAG = 'mavros'
FORCE_REPEL = 0.8  # coefficient of repel
PADDING_ANGLE = -20  # angle in degrees to rotate drone

class Drone:
    drones = []

    def __init__(self, name: str):
        # VECTOR: nav_start --- nav_target >
        current_pos: PoseStamped = rospy.wait_for_message(f"/{name}/local_position/pose", PoseStamped)
        self.nav_start: PoseStamped = current_pos  # initial (start) point of a drone
        self.nav_target: SpeedPoseStamped = SpeedPoseStamped()  # most last point where drone should move
        self.current_pos: PoseStamped = current_pos  # current drone position
        self.setpoint: PoseStamped = current_pos  # the (medium) position where drone should move (ds)

        self.name = name  # drone name
        self.formation_offset: Point = pointify([0, 0, 0])  # offset from formation center

        rospy.Subscriber(f'/{name}/navigate', SpeedPoseStamped, self.cb)
        rospy.Subscriber(f'/{name}/local_position/pose', PoseStamped, self._set_current)
        self.setpoint_pub = rospy.Publisher(f'/{name}/setpoint_position/local', PoseStamped, queue_size=10)

        rospy.ServiceProxy(f"/{name}/set_mode", SetMode)(custom_mode="OFFBOARD")

        start_daemon_thread(target=self._pos_publishing)
        start_daemon_thread(target=self._t_interpolate)

        self.drones.append(self)

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
                inter = self.interpolate()
                repel = self.repel_from_object()
                print(self.name, inter, repel)

                to = PoseStamped()
                to.header.stamp = rospy.Time.now()
                to.pose.position = pointify(inter - repel)
                to.pose.orientation.x = 0.0
                to.pose.orientation.y = 0.0
                to.pose.orientation.z = 0.0
                to.pose.orientation.w = 0.0

                self.setpoint = to
            r.sleep()

    def interpolate(self) -> np.ndarray:
        start = numpyfy(self.nav_start.pose.position)
        target = numpyfy(self.nav_target.position)
        speed = self.nav_target.speed

        distance = np.linalg.norm(target - start)
        time = distance / speed
        passed = min((rospy.get_time() - self.nav_start.header.stamp.to_sec()) / time, 1.0)  # percents
        # print(start, target, passed)

        r = start + (target - start) * passed
        return r

    def cb(self, speed_pose: SpeedPoseStamped):
        if self.setpoint.pose:
            self.nav_start = self.setpoint
        else:
            self.nav_start = self.current_pos
        self.nav_target = speed_pose

    def repel_from_object(self):
        repel_vec = np.array([0.0, 0.0, 0.0])

        course_vec = numpyfy(self.nav_target.position) - numpyfy(self.setpoint.pose.position)
        course = math.atan2(course_vec[1], course_vec[0])

        for drone in self.drones:
            if drone != self:
                self_pos = numpyfy(self.current_pos.pose.position)
                other_pos = numpyfy(drone.current_pos.pose.position)
                # print(self_pos, other_pos)
                vec = other_pos - self_pos
                dist = np.linalg.norm(vec)
                # print(vec, dist)
                if dist < RADIUS:
                    direction = 1.0 if abs(course) <= np.deg2rad(90) else -1.0
                    repel_force = FORCE_REPEL / dist - FORCE_REPEL / RADIUS
                    # print(vec, repel_force)
                    repel_force_vec = np.array([repel_force, repel_force, 0])

                    repel_vec += vec * repel_force_vec

        # print(self.name, repel_vec)
        repel_vec = rotate_vect(repel_vec, np.deg2rad(PADDING_ANGLE))

        return repel_vec
                # if vec[2] == 0.:
                #     repel[2] += max_vel * rep_force(dist) * dir
                # else:
                #     repel[2] += vec[2] * rep_force(dist) * dir


# class Swarm:
#     def __init__(self, drones: List[Drone]):
#         self.drones = drones
#
#     def repel_from_object(self):
#         for drone in self.drones:
#
#

rospy.init_node("position_proxy")



if __name__ == "__main__":
    drones = [Drone(f'{DRONE_TAG}{i}') for i in range(1, n + 1)]
    rospy.spin()
