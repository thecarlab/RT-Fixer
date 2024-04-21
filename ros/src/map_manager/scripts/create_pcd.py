#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import pcl
from pcl_ros import transforms

class PointCloudAccumulator:
    def __init__(self):
        # Create message filters
        cloud_sub = message_filters.Subscriber('/v1/filtered_points', PointCloud2)
        pose_sub = message_filters.Subscriber('/v1/odometry_node/local_pose', PoseStamped)

        # Synchronize messages
        ts = message_filters.ApproximateTimeSynchronizer([cloud_sub, pose_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        # Initialize point cloud list
        self.clouds = []

    def callback(self, cloud_msg, pose_msg):
        # Convert ROS PointCloud2 to PCL PointCloud
        cloud = pcl.PointCloud()
        pcl.fromROSMsg(cloud_msg, cloud)

        # Here you can use pose_msg to transform the cloud if necessary
        # Example: 
        transformed_cloud = transform_cloud(cloud, pose_msg)

        self.clouds.append(cloud)

    def shutdown(self):
        # Merge and downsample point clouds
        merged_cloud = pcl.PointCloud()
        for cloud in self.clouds:
            merged_cloud += cloud

        # Apply Voxel Grid filter
        vg = merged_cloud.make_voxel_grid_filter()
        vg.set_leaf_size(0.01, 0.01, 0.01)  # Set the voxel size
        downsampled = vg.filter()

        # Convert back to ROS message and publish/return/save
        downsampled_ros_msg = pcl.toROSMsg(downsampled)
        # Do something with the downsampled_ros_msg (e.g., publish, save)
        # Save the downsampled point cloud to a PCD file
        pcl.save(downsampled, 'downsampled_cloud.pcd')

# Function to transform cloud using pose data (implement as needed)
        
def transform_cloud(cloud, pose_msg):
    # Create a pcl Transform object
    transform = pcl.Transform()

    # Set the translation and rotation values from the pose message
    transform.translation = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]
    transform.rotation = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]

    # Apply the transformation to the cloud
    transformed_cloud = pcl.PointCloud()
    pcl.transformPointCloud(cloud, transformed_cloud, transform)

    return transformed_cloud

if __name__ == '__main__':
    rospy.init_node('point_cloud_accumulator')
    pca = PointCloudAccumulator()
    rospy.on_shutdown(pca.shutdown)
    rospy.spin()
