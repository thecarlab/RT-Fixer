#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyRequest
import time

def clear_local_map_client():
    rospy.wait_for_service('/v1/clear_local_map')  # Wait for the service to become available
    try:
        clear_map = rospy.ServiceProxy('/v1/clear_local_map', Empty)  # Create a service proxy
        resp = clear_map(EmptyRequest())  # Call the service
        return resp
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('clear_local_map_client_node')

    while not rospy.is_shutdown():
        clear_local_map_client()  # Call the service
        rospy.loginfo("Called /v1/clear_local_map service")
        rospy.sleep(20)  # Sleep for 20 seconds before the next call