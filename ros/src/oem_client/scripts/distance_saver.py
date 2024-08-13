#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Empty
import atexit
import os

# Global list to store distances
distances = []

def distance_callback(msg):
    global distances
    distances.append(msg.data)

def save_to_file():
    # Determine the file name based on existing files
    file_index = 1
    while os.path.exists(f"/mnt/Data/exp/exp2/distance_data_{file_index}.txt"):
        file_index += 1
    file_name = f"/mnt/Data/exp/exp2/distance_data_{file_index}.txt"
    
    with open(file_name, 'w') as f:
        for distance in distances:
            f.write(f"{distance}\n")
    rospy.loginfo(f"Saved distances to {file_name}")

def save_callback(msg):
    rospy.loginfo("Received shutdown signal.")
    save_to_file()
    # rospy.signal_shutdown("Shutdown signal received")

def main():
    rospy.init_node('distance_saver', anonymous=True)
    rospy.Subscriber('/v1/distance', Float64, distance_callback)
    rospy.Subscriber('/save', Empty, save_callback)
    rospy.loginfo("Distance saver node started")

    rospy.spin()

if __name__ == '__main__':
    main()