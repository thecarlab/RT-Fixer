#!/usr/bin/env python3

import rospy
import std_srvs.srv
import subprocess
import os
import time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

class RosbagPlayer:
    def __init__(self):
        # Initialize node
        self.first_time = None
        self.latest_time = None
        rospy.init_node('rosbag_player', anonymous=True)

        # Get parameters
        self.bag_file = rospy.get_param('~bag_file', '')
        if not os.path.exists(self.bag_file):
            rospy.logerr("Specified bag file does not exist.")
            exit(1)
        self.start = rospy.get_param('~start', 0)
        self.end = rospy.get_param('~end', 180)
        self.rate = rospy.get_param('~rate', 1.0)
        self.clock = rospy.get_param('~clock', False)
        self.pause = rospy.get_param('~pause', False)
        self.prefix = rospy.get_param('~prefix', '')  # Prefix for topic namespace

        # Initialize process reference
        self.process = None
        self.time_offset = 0

        # Define service
        self.service1 = rospy.Service('/play_rosbag', std_srvs.srv.Trigger, self.handle_play_bag)
        self.service2 = rospy.Service('/stop_rosbag', std_srvs.srv.Trigger, self.handle_stop_bag)

        # rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.sub = rospy.Subscriber('/v1/last_safe_pose_time', Float64, self.safe_pose_callback)

        self.play()

    def safe_pose_callback(self, msg):
        self.time_offset = msg.data
        rospy.loginfo("\nTime offset: %s\n", str(self.time_offset))

    def clock_callback(self, msg):
        current_time = msg.clock
        
        if self.first_time is None:
            self.first_time = current_time
        
        self.latest_time = current_time
        self.time_offset = (self.latest_time - self.first_time).to_sec()
        rospy.loginfo("Time passed: %s", str(self.time_offset))

    def handle_play_bag(self, req):
        # Start the bag play in a subprocess
        try:
            time.sleep(5)
            self.play()
            return std_srvs.srv.TriggerResponse(True, "Rosbag is playing.")
        except Exception as e:
            return std_srvs.srv.TriggerResponse(False, str(e))
        
    def handle_stop_bag(self, req):
        try:
            self.stop()
            return std_srvs.srv.TriggerResponse(True, "Rosbag stops.")
        except Exception as e:
            return std_srvs.srv.TriggerResponse(False, str(e))
        
    def stop(self):
         # If a rosbag play process is already running, terminate it
        if self.process and self.process.poll() is None:
            rospy.loginfo("Terminating existing rosbag play process.")
            self.process.terminate()
            self.process.wait()  # Ensure the process has terminated before continuing


    def play(self):
        # Build the command to play the rosbag
        command = ['rosbag', 'play', self.bag_file]

        self.start_time = self.start + self.time_offset
        duration = self.end - self.start

        if self.clock:
            command.append('--clock')
        if self.pause:
            command.append('--pause')
        if self.prefix:
            command.extend(['-p', self.prefix])
        if self.start_time:
            command.extend(['-s', str(self.start_time)])
        if self.end:
            command.extend(['-u', str(duration)])
        if self.rate:
            command.extend(['-r', str(self.rate)])

        rospy.loginfo("Playing rosbag: " + ' '.join(command))
        self.process = subprocess.Popen(command)

    def shutdown(self):
        if self.process and self.process.poll() is None:
            rospy.loginfo("Shutting down rosbag play process.")
            self.process.terminate()
            self.process.wait()  # Ensure the process has terminated before exiting

if __name__ == '__main__':
    player = RosbagPlayer()
    rospy.on_shutdown(player.shutdown)
    rospy.spin()
