import rospy
from geometry_msgs.msg import PoseStamped  # message for position/orientation of a drone
from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import Trigger, TriggerResponse
from threading import Thread

n = 6  # number of drones

rospy.init_node("standard_func")

setpoint_local_position_pub = [rospy.Publisher("/mavros{}/setpoint_position/local".format(i), PoseStamped, queue_size=10) for i in range(1, n+1)]
setpoint_local_position = [PoseStamped() for i in range(1, n+1)]

setmode = [rospy.ServiceProxy("/mavros{}/set_mode".format(i), SetMode) for i in range(1, n+1)]
arming = [rospy.ServiceProxy("/mavros{}/cmd/arming".format(i), CommandBool) for i in range(1, n+1)]

stop_thread = False


def send_position():  # threaded func
    global stop_thread
    r = rospy.Rate(10)  # Hz speed of publishing new positions
    while not stop_thread:
        for i in range(n):
            setpoint_local_position_pub[i].publish(setpoint_local_position[i])
        r.sleep()


def start_thread():
    t = Thread(target=send_position)
    t.daemon = True
    t.start()


def land_service(_) -> TriggerResponse:
    for i in range(n):
        t = Thread(target=land_drone, args=(i,))
        t.start()
    return TriggerResponse(success=True, message='Land completed')


def takeoff_service(_) -> TriggerResponse:
    global stop_thread
    start_thread()
    rospy.sleep(1)  # for thread launch
    print('hui')
    for i in range(n):
        t = Thread(target=takeoff_drone, args=(i,))
        t.start()
    rospy.sleep(10)  # waiting for taking off
    stop_thread = True
    return TriggerResponse(success=True, message='Takeoff completed')


def land_drone(i):
    setmode[i](custom_mode="AUTO.LAND")


def takeoff_drone(i):
    current_pose: PoseStamped = rospy.wait_for_message("/mavros{}/local_position/pose".format(i+1), PoseStamped)
    # takeoff on 5 meters
    setpoint_local_position[i].pose.position.x = current_pose.pose.position.x
    setpoint_local_position[i].pose.position.y = current_pose.pose.position.y
    setpoint_local_position[i].pose.position.z = 5.0
    setpoint_local_position[i].pose.orientation.x = 0.0
    setpoint_local_position[i].pose.orientation.y = 0.0
    setpoint_local_position[i].pose.orientation.z = 0.0
    setpoint_local_position[i].pose.orientation.w = 0.0

    rospy.sleep(1)  # msg has arrived

    print(setmode[i](custom_mode="OFFBOARD"))
    print(arming[i](True))



takeoff_s = rospy.Service('/takeoff_all', Trigger, takeoff_service)
land_s = rospy.Service('/land_all', Trigger, land_service)


if __name__ == "__main__":
    rospy.spin()
