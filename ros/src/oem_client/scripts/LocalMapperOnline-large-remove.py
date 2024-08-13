#!/usr/bin/env python3
# Online buffer point clouds and localization
# Generate local map when update is reported.


import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf.transformations import quaternion_matrix
import open3d as o3d
from oem_server.srv import UploadPointCloud, UploadPointCloudRequest

class SyncNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sync_node', anonymous=True)

        # Subscribers with message filters
        
        self.point_cloud_sub = Subscriber('/v1/velodyne_points', PointCloud2)
        # self.point_cloud_sub = Subscriber('/v1/kitti/velo/pointcloud', PointCloud2)
        self.odom_sub = Subscriber('/v1/lidar_odom_pose', Odometry)
        self.ndt_odom_sub = Subscriber('/v1/ndt_pose', Odometry)

        self.map_pub = rospy.Publisher('/v1/merged_point_cloud', PointCloud2, queue_size=10)
        self.map_pub2 = rospy.Publisher('/v1/redundancy_point_cloud', PointCloud2, queue_size=10)


        # Synchronizer with a buffer size of 50
        self.ats = ApproximateTimeSynchronizer(
            [self.point_cloud_sub, self.odom_sub],
            queue_size=10,
            slop=0.1
        )
        self.ats.registerCallback(self.sync_callback)

        # Subscriber for the save topic
        self.save_sub = rospy.Subscriber('/save', Empty, self.save_callback)

        # self.save_sub = Subscriber('/save', Odometry)
        # self.filter_points_sub = Subscriber('/v1/velodyne_points', PointCloud2)

        # # Synchronizer for the new topics
        # self.additional_ats = ApproximateTimeSynchronizer(
        #     [self.save_sub, self.filter_points_sub],
        #     queue_size=10,
        #     slop=0.1
        # )
        # self.additional_ats.registerCallback(self.save_callback)

        self.additional_ats = ApproximateTimeSynchronizer(
            [self.point_cloud_sub, self.ndt_odom_sub],
            queue_size=10,
            slop=0.1
        )
        self.additional_ats.registerCallback(self.sync_callback2)

        # Buffers for synchronized messages
        self.lidar_odom_buffer = []
        self.ndt_odom_buffer = []
        self.save_triggered = False

    def sync_callback(self, point_cloud, odom):
        # Callback function for synchronized messages
        rospy.loginfo(f"lidar odom buffer size {len(self.lidar_odom_buffer)}")

        if not self.save_triggered:
            # Add new messages to the pre-save buffer
            self.lidar_odom_buffer.append((point_cloud, odom))
            if len(self.lidar_odom_buffer) > 40:
                self.lidar_odom_buffer.pop(0)

    def sync_callback2(self, point_cloud, odom):
        # Callback function for synchronized messages
        rospy.loginfo(f"ndt odom buffer size {len(self.ndt_odom_buffer)}")

        if not self.save_triggered:
            # Add new messages to the pre-save buffer
            self.ndt_odom_buffer.append((point_cloud, odom))
            if len(self.ndt_odom_buffer) > 10:
                self.ndt_odom_buffer.pop(0)

    # def save_callback(self, ndt_pose, filter_points):
    #     # Callback function for save topic
    #     rospy.loginfo("Save message received")
    #     self.save_triggered = False
    #     self.post_save_buffer = []

    #     self.redundancy_pose = ndt_pose
    #     self.redundancy_cloud = filter_points
    #     self.generateLocalMap()

    def save_callback(self, emptyMsg):
        # Callback function for save topic
        rospy.loginfo("Save message received")
        self.generateLocalMap()

    def transform_point_cloud(self, point_cloud, odom):
        # Extract pose information from odometry
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        transform1 = quaternion_matrix([0, 0, 0, 1])
        transform1[0:3, 3] = [2.0, 0.0, 1.8]

        # Create transformation matrix
        transform = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        transform[0:3, 3] = [position.x, position.y, position.z]

        combined_transform = np.dot(transform, transform1)

        # Transform point cloud
        points = np.array(list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)))
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = np.dot(combined_transform, points_homogeneous.T).T

        return transformed_points[:, 0:3]

    def save_point_cloud(self, points, filename):
        # Convert numpy array to open3d point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(filename, cloud)

    def generateLocalMap(self):
        rospy.loginfo("Generating local map...")

        def downsample_point_cloud(points, voxel_size=0.1):
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            downsampled_cloud = cloud.voxel_down_sample(voxel_size)
            return np.asarray(downsampled_cloud.points)

        
        def merge_point_clouds(buffers, N):
            combined_points = []
            for point_cloud, odom in buffers[-N:]:
                transformed_points = self.transform_point_cloud(point_cloud, odom)
                combined_points.append(transformed_points)
            return np.vstack(combined_points)
        
        def create_point_cloud_msg(points):
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'global_map'  # Adjust this frame_id as necessary
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]
            return pc2.create_cloud(header, fields, points)

        point_cloud_msg = PointCloud2()

        # Generate five maps with different N values
        for N in [25]:
            # if len(self.pre_save_buffer) >= N and len(self.post_save_buffer) >= N:
            combined_points = merge_point_clouds(self.lidar_odom_buffer, N)
            downsampled_points = downsample_point_cloud(combined_points, 1.0)
            point_cloud_msg = create_point_cloud_msg(downsampled_points)
            #pcd_filename = f"/mnt/Data/exp/exp3/merged_map_N{N}.pcd"
            # self.save_point_cloud(combined_points, pcd_filename)
            # rospy.loginfo(f"Generated map with N={N} and saved to {pcd_filename}")

        rospy.loginfo("get latest local_map")

        for N in [10]:
            # if len(self.pre_save_buffer) >= N and len(self.post_save_buffer) >= N:
            combined_points = merge_point_clouds(self.ndt_odom_buffer, N)
            # downsampled_points = downsample_point_cloud(combined_points, 0.2)
            transformed_redundancy_cloud_msg = create_point_cloud_msg(combined_points)
            #pcd_filename = f"/mnt/Data/exp/exp3/merged_map_N{N}.pcd"
            # self.save_point_cloud(combined_points, pcd_filename)
            # rospy.loginfo(f"Generated map with N={N} and saved to {pcd_filename}")

        rospy.loginfo("get redundancy")
        self.map_pub.publish(point_cloud_msg)
        self.map_pub2.publish(transformed_redundancy_cloud_msg)
        self.lidar_odom_buffer = []
        self.ndt_odom_buffer = []
        self.save_triggered = False

        rospy.loginfo(point_cloud_msg.header)
        rospy.loginfo(transformed_redundancy_cloud_msg.header)

        try:
            upload_point_cloud = rospy.ServiceProxy('/upload_point_cloud', UploadPointCloud)
            update_req = UploadPointCloudRequest()
            update_req.cloud = point_cloud_msg
            update_req.redundancy_cloud = transformed_redundancy_cloud_msg
            resp = upload_point_cloud(update_req)
            rospy.loginfo(f"Service call successful")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        try:
            play_rosbag  = rospy.ServiceProxy('/play_rosbag', Trigger)
            req = TriggerRequest()
            response = play_rosbag(req)
            rospy.loginfo(f"Restart rosbag successful")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        sync_node = SyncNode()
        sync_node.run()
    except rospy.ROSInterruptException:
        pass
