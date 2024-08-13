#!/usr/bin/env python3
# Online buffer point clouds and localization
# Generate local map when update is reported.


import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf.transformations import quaternion_matrix
import open3d as o3d

class SyncNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sync_node', anonymous=True)

        # Subscribers with message filters
        self.point_cloud_sub = Subscriber('/v1/velodyne_points', PointCloud2)
        self.odom_sub = Subscriber('/v1/lidar_odom_pose', Odometry)

        # Synchronizer with a buffer size of 50
        self.ats = ApproximateTimeSynchronizer(
            [self.point_cloud_sub, self.odom_sub],
            queue_size=50,
            slop=0.1
        )
        self.ats.registerCallback(self.sync_callback)

        # Subscriber for the save topic
        self.save_sub = rospy.Subscriber('/save', Empty, self.save_callback)

        # Buffers for synchronized messages
        self.pre_save_buffer = []
        self.post_save_buffer = []
        self.save_triggered = False

    def sync_callback(self, point_cloud, odom):
        # Callback function for synchronized messages
        rospy.loginfo("Synchronized messages received")

        if not self.save_triggered:
            # Add new messages to the pre-save buffer
            self.pre_save_buffer.append((point_cloud, odom))
            if len(self.pre_save_buffer) > 25:
                self.pre_save_buffer.pop(0)
        else:
            # Add new messages to the post-save buffer
            self.post_save_buffer.append((point_cloud, odom))
            if len(self.post_save_buffer) >= 25:
                self.generateLocalMap()
                rospy.signal_shutdown("Saved 25 frames and generated local map")

    def save_callback(self, msg):
        # Callback function for save topic
        rospy.loginfo("Save message received")
        self.save_triggered = True
        self.post_save_buffer = []

    def transform_point_cloud(self, point_cloud, odom):
        # Extract pose information from odometry
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        # Create transformation matrix
        transform = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        transform[0:3, 3] = [position.x, position.y, position.z]

        # Transform point cloud
        points = np.array(list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)))
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = np.dot(transform, points_homogeneous.T).T

        return transformed_points[:, 0:3]

    def save_point_cloud(self, points, filename):
        # Convert numpy array to open3d point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(filename, cloud)

    def save_buffers_to_file(self, filename):
        with open(filename, 'w') as f:
            f.write("Pre-save buffer:\n")
            for point_cloud, odom in self.pre_save_buffer:
                f.write(f"PointCloud: {point_cloud.header.stamp.to_sec()} | Odometry: {odom.header.stamp.to_sec()}\n")

            f.write("\nPost-save buffer:\n")
            for point_cloud, odom in self.post_save_buffer:
                f.write(f"PointCloud: {point_cloud.header.stamp.to_sec()} | Odometry: {odom.header.stamp.to_sec()}\n")

    def generateLocalMap(self):
        rospy.loginfo("Generating local map...")
        
        def merge_point_clouds(buffers, N):
            combined_points = []
            for point_cloud, odom in buffers[-N:]:
                transformed_points = self.transform_point_cloud(point_cloud, odom)
                combined_points.append(transformed_points)
            for point_cloud, odom in self.post_save_buffer[:N]:
                transformed_points = self.transform_point_cloud(point_cloud, odom)
                combined_points.append(transformed_points)
            return np.vstack(combined_points)

        # Generate five maps with different N values
        for N in [5, 10, 15, 20, 25]:
            if len(self.pre_save_buffer) >= N and len(self.post_save_buffer) >= N:
                combined_points = merge_point_clouds(self.pre_save_buffer, N)
                pcd_filename = f"/mnt/Data/exp/exp3/merged_map_N{N}.pcd"
                self.save_point_cloud(combined_points, pcd_filename)
                rospy.loginfo(f"Generated map with N={N} and saved to {pcd_filename}")

        # Save buffers to file
        buffers_filename = "/mnt/Data/exp/exp3/buffers.txt"
        self.save_buffers_to_file(buffers_filename)
        rospy.loginfo(f"Buffers saved to {buffers_filename}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        sync_node = SyncNode()
        sync_node.run()
    except rospy.ROSInterruptException:
        pass
