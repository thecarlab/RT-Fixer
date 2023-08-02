#!/usr/bin/env python3

import os
import rospy
import rosbag
import argparse
from std_msgs.msg import String


def play_rosbag(bag_path):
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/hesai/pandar']):
            if rospy.is_shutdown():
                break
            pub = publishers.get(topic)
            if pub is None:
                pub = rospy.Publisher(topic, msg.__class__, queue_size=1)
                publishers[topic] = pub
            pub.publish(msg)
            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('rosbag_player', anonymous=True)

    parser = argparse.ArgumentParser(description='Play rosbags with a given name pattern consecutively.')
    parser.add_argument('-d', '--directory', required=True, help='Directory containing the rosbag files.')
    args = parser.parse_args()

    bag_name_pattern = "20230505_LabnSurrounding_Pandar64_{}.bag"
    publishers = {}

    bag_id = 32 
    total_bags = 50 

    while not rospy.is_shutdown() and bag_id < total_bags:
        bag_name = bag_name_pattern.format(bag_id)
        bag_path = os.path.join(args.directory, bag_name)

        if not os.path.exists(bag_path):
            print(f"Rosbag file {bag_name} not found. Skipping.")
            bag_id += 1
            continue

        print("Playing rosbag:", bag_path)
        play_rosbag(bag_path)

        bag_id += 1
    
    print("All rosbags have been played.")