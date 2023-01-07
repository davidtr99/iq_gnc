#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
from geometry_msgs.msg import Point

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    takeoff_height = rospy.get_param('~takeoff_height')

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    #drone.wait4start()
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(takeoff_height)
   
    cur_pose = Point()
    # Getting the current location from the drone.
    cur_pose = drone.get_current_location()
    rospy.loginfo(
            "Pose-> x:{} y:{} z:{} ".format(cur_pose.x, cur_pose.y, cur_pose.z))
    rospy.loginfo(CGREEN2 + "drone spawned" + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
