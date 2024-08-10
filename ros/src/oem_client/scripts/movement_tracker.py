#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix, quaternion_conjugate, concatenate_matrices, translation_matrix, translation_from_matrix
import tf
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyRequest
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Empty as MsgEmpty
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_geometry_msgs
import message_filters
from std_msgs.msg import Float64, Float32MultiArray, Header
import numpy as np
import threading
from oem_server.srv import UploadPointCloud, UploadPointCloudRequest
from geometry_msgs.msg import Pose
from time import sleep
from rospy.exceptions import ROSTimeMovedBackwardsException
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion


import threading

def inverse_transform(transform):
    # Extract the current translation and rotation
    translation = np.array([transform.transform.translation.x, 
                            transform.transform.translation.y, 
                            transform.transform.translation.z, 1])  # Make it a homogenous coordinate for matrix operations
    
    rotation = np.array([transform.transform.rotation.x, 
                         transform.transform.rotation.y, 
                         transform.transform.rotation.z, 
                         transform.transform.rotation.w])
    
    # Invert the rotation using quaternion conjugate
    rotation_inv = quaternion_conjugate(rotation)
    
    # Convert the inverted quaternion to a rotation matrix
    rotation_inv_matrix = quaternion_matrix(rotation_inv)
    
    # Negate the translation and transform by the inverted rotation matrix
    # Apply the rotation matrix to the translation vector
    translation_inv = np.dot(rotation_inv_matrix, -translation)[:3]  # Apply matrix, and take only the x, y, z components
    
    # Create a new TransformStamped for the inverse transformation
    inv_transform = TransformStamped()
    inv_transform.transform.translation.x, inv_transform.transform.translation.y, inv_transform.transform.translation.z = translation_inv
    inv_transform.transform.rotation.x, inv_transform.transform.rotation.y, inv_transform.transform.rotation.z, inv_transform.transform.rotation.w = rotation_inv
    
    return inv_transform

def transform_odometry(odometry_msg, translation, rotation_angle_deg):
    # Convert rotation angle from degrees to radians
    rotation_angle_rad = -rotation_angle_deg * (3.14159265 / 180.0)  # Negative for clockwise rotation

    # Create a quaternion from the Euler angles
    q_rot = tf.transformations.quaternion_from_euler(0, 0, rotation_angle_rad)
    
    # Apply translation
    odometry_msg.pose.pose.position.x += translation[0]
    odometry_msg.pose.pose.position.y += translation[1]
    odometry_msg.pose.pose.position.z += translation[2]
    
    # Apply rotation
    original_orientation = odometry_msg.pose.pose.orientation
    original_quaternion = (original_orientation.x,
                           original_orientation.y,
                           original_orientation.z,
                           original_orientation.w)
    new_quaternion = tf.transformations.quaternion_multiply(q_rot, original_quaternion)
    
    # Update the odometry message
    odometry_msg.pose.pose.orientation = Quaternion(*new_quaternion)
    
    return odometry_msg

def transform_to_pose(transform):
    # Create a Pose message
    pose = Pose()
    
    # Copy the translation components
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    
    # Copy the rotation components
    pose.orientation.x = transform.transform.rotation.x
    pose.orientation.y = transform.transform.rotation.y
    pose.orientation.z = transform.transform.rotation.z
    pose.orientation.w = transform.transform.rotation.w
    
    return pose

def pose_to_transform(pose):
    
    t = TransformStamped()
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    return t

def transform_to_matrix(trans):
    """Convert TransformStamped to 4x4 transformation matrix."""
    translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    rotation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(rotation))

def matrix_to_transform(matrix):
    """Convert 4x4 transformation matrix to TransformStamped."""
    trans = TransformStamped()
    trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z = translation_from_matrix(matrix)
    trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w = quaternion_from_matrix(matrix)
    return trans

def multiply_transforms(trans1, quat1, trans2, quat2):
    # Convert quaternion to a rotation matrix
    mat1 = quaternion_matrix(quat1)
    mat1[:3, 3] = trans1
    
    mat2 = quaternion_matrix(quat2)
    mat2[:3, 3] = trans2
    
    # Matrix multiplication
    mat_combined = np.dot(mat1, mat2)
    
    # Extract the translation and quaternion from the combined matrix
    trans_combined = mat_combined[:3, 3]
    quat_combined = quaternion_from_matrix(mat_combined)
    
    return trans_combined, quat_combined

class MovementTracker:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ndt_movement_tracker', anonymous=True)
        
        # Get topic names from parameters
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.local_map_frame = rospy.get_param('~local_map_frame', 'local_map_frame')
        self.local_odom_frame = rospy.get_param('~local_odom_frame', 'local_odom_frame')
        self.map_shift_thresh = rospy.get_param('~mapShiftThresh', 40)
        self.odom_refresh_thresh = rospy.get_param('~odomRefreshThresh', 40)
        self.publish_hz = rospy.get_param('~map_odometry_tf_hz', 100)

        print("Movement Tracker Parameters:")
        print("  Base Frame: {}".format(self.base_frame))
        print("  Odometry Frame: {}".format(self.local_map_frame))
        print("  Map Shift Threshold: {} units".format(self.map_shift_thresh))
        print("  Odometry Refresh Threshold: {} units".format(self.odom_refresh_thresh))
        print("  Publish Frequency: {} Hz".format(self.publish_hz))

        self.base_link_odom = Odometry()
        self.base_link_odom.header.frame_id = "v1/local_sensor"

        self.base_link_odom.pose.pose.position.x = rospy.get_param('~sensor_base_x', -2.0)
        self.base_link_odom.pose.pose.position.y = rospy.get_param('~sensor_base_y', 0.0)
        self.base_link_odom.pose.pose.position.z = rospy.get_param('~sensor_base_z', -1.8)

        self.base_link_odom.pose.pose.orientation.x = rospy.get_param('~sensor_base_qx', 0.0)
        self.base_link_odom.pose.pose.orientation.y = rospy.get_param('~sensor_base_qy', 0.0)
        self.base_link_odom.pose.pose.orientation.z = rospy.get_param('~sensor_base_qz', 0.0)
        self.base_link_odom.pose.pose.orientation.w = rospy.get_param('~sensor_base_qw', 1.0)

        trans_ab = TransformStamped()
        trans_ab.transform.translation.x = self.base_link_odom.pose.pose.position.x
        trans_ab.transform.translation.y = self.base_link_odom.pose.pose.position.y
        trans_ab.transform.translation.z = self.base_link_odom.pose.pose.position.z
        trans_ab.transform.rotation.x = self.base_link_odom.pose.pose.orientation.x
        trans_ab.transform.rotation.y = self.base_link_odom.pose.pose.orientation.y
        trans_ab.transform.rotation.z = self.base_link_odom.pose.pose.orientation.z
        trans_ab.transform.rotation.w = self.base_link_odom.pose.pose.orientation.w

        self.trans_local_map_to_local_odom = inverse_transform(trans_ab)
        self.br = tf2_ros.TransformBroadcaster()

        self.deviation_buf = []
        self.pre_ndt_pose = None
        self.pre_lidar_odom_pose = None

        self.initialized = False

        self.found_update = False
        self.pause_system = False
        self.update_local_odom = True

        self.before_update_count = 0
        self.current_safe_pose = None
        self.last_safe_pose = None
        self.first_pose = None
        self.odom_update_count = 0

        self.lidar_odom_movement_meter_for_submap = 0
        self.lidar_odom_movement_meter_for_localSLAM = 0
        self.ndt_odom_movement_meter = 0
        self.max_distance = -1

        self.deviation_pub = rospy.Publisher('distance', Float64, queue_size=10)
        self.orientation_deviation_pub = rospy.Publisher('angle', Float64, queue_size=10)
        self.lidar_odom_movement_pub = rospy.Publisher('lidar_odom_movement', Float64, queue_size=10)
        self.ndt_odom_movement_pub = rospy.Publisher('ndt_odom_movement', Float64, queue_size=10)

        self.initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.switchmap_pub = rospy.Publisher('/switchMap', Odometry, queue_size=10)
        self.lidar_odom_pub = rospy.Publisher('lidar_odom_pose', Odometry, queue_size=10)
        self.relocalization_pub = rospy.Publisher('relocalization', MsgEmpty, queue_size=10)

        self.save_pub = rospy.Publisher('/save', MsgEmpty, queue_size=10)
       
        self.initialpose_sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initialpose_callback)
        self.initialpose_sub2 = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)

        self.last_safe_pose_time_pub = rospy.Publisher('last_safe_pose_time', Float64, queue_size=10)
        
        # self.local_map_sub = rospy.Subscriber("/v1/odometry_node/local_map", PointCloud2, self.local_map_callback)


        self.sub1 = message_filters.Subscriber('ndt_pose', Odometry)
        self.sub2 = message_filters.Subscriber('/v1/odometry_node/odometry', Odometry)
        self.sub3 = message_filters.Subscriber('/v1/odometry_node/local_map', PointCloud2)
        
        self.ts2 =  message_filters.TimeSynchronizer([self.sub1, self.sub2, self.sub3], 1)
        self.ts2.registerCallback(self.test)

    # def local_map_callback(self, map_msg):
    #     if trans_global_map_to_local_odom is not None:
    #         transformed_local_map = do_transform_cloud(map_msg, trans_global_map_to_local_odom)
    #         transformed_local_map.header = map_msg.header
    #         transformed_local_map.header.frame_id = "global_map"
    #         self.local_map_pub.publish(transformed_local_map)

    # def local_map_callback(self, map_msg):
        
    #     self.latest_local_map_buf.append(map_msg)

    #     if len(self.latest_local_map_buf) > 10:
    #         self.latest_local_map_buf.pop(0)

    def cal_movement(self, pre_pose, cur_pose):
        
        dx = cur_pose.pose.position.x - pre_pose.pose.position.x
        dy = cur_pose.pose.position.y - pre_pose.pose.position.y

        return math.sqrt(dx**2 + dy**2)
    
    def calculate_orientation_difference(self, pose1, pose2):
        # Extract quaternions from the poses
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        
        # Calculate the inverse of the first quaternion
        q1_inv = quaternion_inverse(q1)
        
        # Calculate the relative rotation between the two quaternions
        q_diff = quaternion_multiply(q2, q1_inv)
        
        # Convert the resulting quaternion to Euler angles
        euler_diff = euler_from_quaternion(q_diff)
        
        # Convert Euler angles from radians to degrees
        euler_diff_degrees = [angle * 180.0 / np.pi for angle in euler_diff]
        
        return euler_diff_degrees
    
    def test(self, ndt_pose_msg, lidar_odom_msg, local_map_msg):
    
        if self.pause_system:
            rospy.loginfo("Waiting update, pause system")
            return
        
        rospy.loginfo(f"current odom update times {self.odom_update_count}")
        
            
        if self.first_pose is None:
            self.first_pose = ndt_pose_msg


        # rospy.loginfo(f"test, ndt_pose_id: {ndt_pose_msg.header.seq}, lidar_odom_msg_id: {lidar_odom_msg.header.seq}, local_map_msg_id: {local_map_msg.header.seq}")
        # rospy.loginfo(f"test, ndt_pose_id: {ndt_pose_msg.header.stamp}, lidar_odom_msg_id: {lidar_odom_msg.header.stamp}, local_map_msg_id: {local_map_msg.header.stamp}")
        if self.pre_ndt_pose is not None:
            ndt_movement = self.cal_movement(self.pre_ndt_pose, ndt_pose_msg.pose)
            self.ndt_odom_movement_meter += ndt_movement
            # rospy.loginfo(f"ndt_movement: {ndt_movement}")
            # self.ndt_odom_movement_pub.publish(math.log(self.ndt_odom_movement_meter, 10))
            self.ndt_odom_movement_pub.publish(self.ndt_odom_movement_meter)

        rospy.loginfo(f"Update local odom {self.update_local_odom}")
        if self.update_local_odom:
            self.current_safe_pose_time = ndt_pose_msg.header.stamp
            self.current_safe_pose =ndt_pose_msg
            self.odom_pose = ndt_pose_msg.pose.pose
            self.global_map_to_local_map_frame = pose_to_transform(self.odom_pose)
            self.update_local_odom = False
            rospy.loginfo(f"ndt_pose_msg {ndt_pose_msg}")
            rospy.loginfo(f"lidar_odom_msg {lidar_odom_msg}")

        self.local_odom_to_local_sensor = pose_to_transform(lidar_odom_msg.pose.pose)

        # Convert all transformations to matrices
        mat_ab = transform_to_matrix(self.global_map_to_local_map_frame)
        mat_bc = transform_to_matrix(self.trans_local_map_to_local_odom)
        mat_cd = transform_to_matrix(pose_to_transform(lidar_odom_msg.pose.pose))

        # Combine transformations by matrix multiplication
        mat_ac = np.dot(mat_ab, mat_bc)
        mat_ad = np.dot(mat_ac, mat_cd)
        trans_global_map_to_local_odom = matrix_to_transform(mat_ac)
        trans_global_map_to_local_sensor = matrix_to_transform(mat_ad)
        # transformed_local_map = do_transform_cloud(local_map_msg, trans_global_map_to_local_odom)
        transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=self.base_link_odom.header, pose=self.base_link_odom.pose.pose), trans_global_map_to_local_sensor)
        transformed_odom = Odometry()
        transformed_odom.header.stamp = lidar_odom_msg.header.stamp
        transformed_odom.header.frame_id = "global_map"
        transformed_odom.pose.pose = transformed_pose.pose
        # transformed_odom.pose.pose.position.x = ndt_pose_msg.pose.pose.position.x
        # transformed_odom.pose.pose.position.y = ndt_pose_msg.pose.pose.position.y
        transformed_odom.pose.pose.position.z = ndt_pose_msg.pose.pose.position.z
        self.lidar_odom_pub.publish(transformed_odom)

        # transformed_local_map.header = local_map_msg.header
        # transformed_local_map.header.frame_id = "global_map"
        # self.local_map_pub.publish(transformed_local_map)
        trans_global_map_to_local_odom.header.stamp = local_map_msg.header.stamp
        trans_global_map_to_local_odom.header.frame_id = "global_map"
        trans_global_map_to_local_odom.child_frame_id = local_map_msg.header.frame_id

        self.br.sendTransform(trans_global_map_to_local_odom) 

        pos1 = ndt_pose_msg.pose.pose.position
        pos2 = transformed_odom.pose.pose.position
        deviation = ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2) ** 0.5
        orientation_diff  = self.calculate_orientation_difference(ndt_pose_msg.pose.pose, transformed_odom.pose.pose)
        print(f"Orientation difference (in degrees): {orientation_diff}")
        orientation_diff_msg = Float32MultiArray(data=orientation_diff)
        self.orientation_deviation_pub.publish(orientation_diff[1])
        self.deviation_buf.append(deviation)
        self.deviation_pub.publish(deviation)
        if not self.found_update:
            if deviation > 1000000.0:
                
                if self.odom_update_count < 1:
                    rospy.loginfo('Map incorrect from beginning')
                    # rospy.signal_shutdown('Map incorrect from beginning')

                self.found_update = True
                self.local_map_pose = self.odom_pose
                # empty_msg = MsgEmpty()
                # self.save_pub.publish(empty_msg)
                rospy.loginfo("Calling generate local map")
                #self.save_pub.publish(ndt_pose_msg)
                # 
                # rospy.loginfo("Calling generate local map")
                # self.save_pub.publish(ndt_pose_msg)

        # if len(self.deviation_buf) > 3 and not self.found_update:
        #     self.deviation_buf.pop(0)
        #     mean_deviation = sum(self.deviation_buf) / 3
        #     self.deviation_pub.publish(mean_deviation)

        #     if mean_deviation > 100000:
        #         self.found_update = True
        #         self.local_map_pose = self.odom_pose
                
        #         empty_msg = MsgEmpty()
        #         self.save_pub.publish(empty_msg)
        #         rospy.loginfo("Calling generate local map")
        #         #self.save_pub.publish(ndt_pose_msg)

        if self.found_update:
            # rospy.loginfo(f"Published {self.before_update_count} message after update found")
            # if self.before_update_count < 30:
            #     self.before_update_count += 1
            # else:
            #     self.pause_system = True

            #     try:
            #         play_rosbag  = rospy.ServiceProxy('/stop_rosbag', Trigger)
            #         req = TriggerRequest()
            #         response = play_rosbag(req)
            #         rospy.loginfo(f"stop rosbag successful")
            #     except rospy.ServiceException as e:
            #         rospy.logerr("Service call failed: %s" % e)

            #     # Wait, until all message in queue is processed.
            #     sleep(3)

            #     self.found_update = False
            #     self.latest_local_map = local_map_msg
            #     self.latest_trans2 = trans_global_map_to_local_odom

            #     rospy.loginfo("Start Call Map Merge") 
            #     thread = threading.Thread(target=self.call_map_merge_service, args=())
            #     thread.start()

            if self.pre_lidar_odom_pose is not None:
                lidar_odom_movement = self.cal_movement(self.pre_lidar_odom_pose, transformed_odom.pose)
                # rospy.loginfo(f"lidar_movement: {lidar_odom_movement}")
                self.lidar_odom_movement_meter_for_localSLAM += lidar_odom_movement
                if self.lidar_odom_movement_meter_for_localSLAM >= self.odom_refresh_thresh:
                    # rospy.loginfo("Start Call Map Merge") 
                    # thread = threading.Thread(target=self.call_map_merge_service, args=())
                    # thread.start()
                    self.pause_system = True

                    try:
                        play_rosbag  = rospy.ServiceProxy('/stop_rosbag', Trigger)
                        req = TriggerRequest()
                        response = play_rosbag(req)
                        rospy.loginfo(f"stop rosbag successful")
                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s" % e)

                    # Wait, until all message in queue is processed.
                    sleep(3)

                    rospy.loginfo("Calling generate local map")
                    empty_msg = MsgEmpty()
                    self.save_pub.publish(empty_msg)

                    time_difference = (self.initial_pose_msg.header.stamp - self.first_pose.header.stamp).to_sec()
                    self.last_safe_pose_time_pub.publish(time_difference)
                    # sleep(1)

                    self.found_update = False
                    self.latest_local_map = local_map_msg
                    self.latest_trans2 = trans_global_map_to_local_odom

                    rospy.loginfo("Start Call Map Merge") 
                    thread = threading.Thread(target=self.call_map_merge_service, args=())
                    thread.start()

            return

        # rospy.loginfo(f"check lidar pose {self.pre_lidar_odom_pose}")
        if self.pre_lidar_odom_pose is not None:
            lidar_odom_movement = self.cal_movement(self.pre_lidar_odom_pose, transformed_odom.pose)
            # rospy.loginfo(f"lidar_movement: {lidar_odom_movement}")
            self.lidar_odom_movement_meter_for_submap += lidar_odom_movement
            self.lidar_odom_movement_meter_for_localSLAM += lidar_odom_movement

            # self.lidar_odom_movement_pub.publish(math.log(self.lidar_odom_movement_meter_for_localSLAM, 10))
            self.lidar_odom_movement_pub.publish(math.log(self.lidar_odom_movement_meter_for_localSLAM, 10))
            if self.lidar_odom_movement_meter_for_submap >= self.map_shift_thresh:
                odometry_msg = Odometry()
                odometry_msg.header.stamp = lidar_odom_msg.header.stamp
                odometry_msg.header.frame_id = "global_map" 
                odometry_msg.child_frame_id = self.base_frame
                odometry_msg.pose = transformed_odom.pose
                self.switchmap_pub.publish(odometry_msg)
                self.lidar_odom_movement_meter_for_submap -=  self.map_shift_thresh
                empty_msg = MsgEmpty()
                # rospy.loginfo("Calling relocalization after certain movement")
                # self.relocalization_pub.publish(empty_msg)
                # temp = PoseWithCovarianceStamped()
                # temp.header = transformed_odom.header

                # Copy pose and covariance
                # temp.pose.pose = transformed_odom.pose.pose
                # temp.pose.covariance = transformed_odom.pose.covariance
                # self.initialpose_pub.publish(temp)

            if self.lidar_odom_movement_meter_for_localSLAM >= self.odom_refresh_thresh:
                self.ndt_odom_movement_meter -= self.odom_refresh_thresh

                if self.ndt_odom_movement_meter < 0:
                    self.ndt_odom_movement_meter = 0
                self.lidar_odom_movement_meter_for_localSLAM -= self.odom_refresh_thresh
                self.update_local_odom = True
                
                

                if self.odom_update_count < 1:
                    self.initial_pose_msg.header = self.first_pose.header
                    self.initial_pose_msg.pose = self.first_pose.pose
                else:
                    self.initial_pose_msg.header = self.last_safe_pose.header
                    self.initial_pose_msg.pose = self.last_safe_pose.pose
                
                self.last_safe_pose = self.current_safe_pose
                self.odom_update_count += 1

                # if self.last_safe_pose is not None:
                #     self.initial_pose_msg.header = self.last_safe_pose.header
                #     self.initial_pose_msg.pose = self.last_safe_pose.pose
                #     if self.last_safe_pose_time is None:
                #         self.last_safe_pose_time = self.first_pose_time
                #     else:
                #         self.last_safe_pose_time = self.current_safe_pose_time
                #     self.last_safe_pose = self.current_safe_pose
                # else:
                #     self.last_safe_pose = self.first_pose

                try:
                    clear_map = rospy.ServiceProxy('/v1/clear_local_map', Empty)  # Create a service proxy
                    resp = clear_map(EmptyRequest())  # Call the service
                    print(resp)
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: %s", e)
                now = rospy.get_rostime()
                rospy.loginfo(f"***local odom update time: secs {now.secs}, nsecs {now.nsecs}")
                self.deviation_buf = []

                # if self.found_update:
                    
                #     self.found_update = False
                #     self.pause_system = True
                #     self.latest_local_map = local_map_msg
                #     self.latest_trans2 = trans_global_map_to_local_odom

                #     try:
                #         play_rosbag  = rospy.ServiceProxy('/stop_rosbag', Trigger)
                #         req = TriggerRequest()
                #         response = play_rosbag(req)
                #         rospy.loginfo(f"Restart rosbag successful")
                #     except rospy.ServiceException as e:
                #         rospy.logerr("Service call failed: %s" % e)

                #     rospy.loginfo("Start Call Map Merge") 
                #     thread = threading.Thread(target=self.call_map_merge_service, args=())
                #     thread.start()
                # else:
                #     try:
                #         clear_map = rospy.ServiceProxy('/v1/clear_local_map', Empty)  # Create a service proxy
                #         resp = clear_map(EmptyRequest())  # Call the service
                #         print(resp)
                #     except rospy.ServiceException as e:
                #         rospy.logwarn("Service call failed: %s", e)
                #     now = rospy.get_rostime()
                #     rospy.loginfo(f"***local odom update time: secs {now.secs}, nsecs {now.nsecs}")
                #     self.deviation_buf = []

        self.pre_ndt_pose = ndt_pose_msg.pose
        self.pre_lidar_odom_pose = transformed_odom.pose


    def call_map_merge_service(self):
        
        # latest_local_map = rospy.wait_for_message("/v1/odometry_node/local_map", PointCloud2)

        # try:
        #     trans2 = self.tf_buffer.lookup_transform("global_map",
        #                                             latest_local_map.header.frame_id,  
        #                                             latest_local_map.header.stamp,
        #                                             rospy.Duration(1.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logwarn("Failed to get transform from {} to {}: {}".format("global_map", latest_local_map.header.frame_id, e))

        # map_msg = self.latest_local_map
        # trans2 = self.latest_trans2
        
        # rospy.loginfo("get latest local_map")
        # try:
        #     upload_point_cloud = rospy.ServiceProxy('/upload_point_cloud', UploadPointCloud)
        #     resp = upload_point_cloud(map_msg, transform_to_pose(trans2))
        #     rospy.loginfo(f"Service call successful")
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s" % e)

        # Turn of TF publisher before restarting system
        # self.tf_publish_timer.shutdown()
        # self.ifPublishTF = False
        # self.update_local_odom = False
        
        try:
            clear_map = rospy.ServiceProxy('/v1/clear_local_map', Empty)  # Create a service proxy
            resp = clear_map(EmptyRequest())  # Call the service
            print(resp)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s", e)

        # sleep(2)
        # self.tf_publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.publish_transform)
        # self.last_callback_time = rospy.Time(0)
        # rospy.sleep(2)
        self.deviation_buf = []
        self.pre_ndt_pose = None
        self.pre_lidar_odom_pose = None

        self.initialized = False

        self.found_update = False
        self.pause_system = False
        self.update_local_odom = True

        self.before_update_count = 0
        self.current_safe_pose = None
        self.last_safe_pose = None
        self.odom_update_count = 0
        # self.first_pose_time = None

        self.lidar_odom_movement_meter_for_submap = 0
        self.lidar_odom_movement_meter_for_localSLAM = 0
        self.ndt_odom_movement_meter = 0
        trans_global_map_to_local_odom = None
        self.pause_system = False
        self.max_distance = -1

        self.initialpose_pub.publish(self.initial_pose_msg)
        # empty_msg = MsgEmpty()
        # rospy.loginfo("Calling relocalization after map merge")
        # self.relocalization_pub.publish(empty_msg)
       
        
        # try:
        #     play_rosbag  = rospy.ServiceProxy('/play_rosbag', Trigger)
        #     req = TriggerRequest()
        #     response = play_rosbag(req)
        #     rospy.loginfo(f"Restart rosbag successful")
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s" % e)

        return True
    
    # def initialpose_callback2(self, msg):
    #     # Process initial pose message
    #     rospy.loginfo(f"Received initial pose: secs {msg.header.stamp.secs}, nsecs {msg.header.stamp.nsecs}")
    #     self.initial_pose_msg = msg

    #     # Convert initial pose to Odometry message
    #     odometry_msg = Odometry()
    #     odometry_msg.header.stamp = rospy.Time.now()
    #     odometry_msg.header.frame_id = "global_map" # Or "global_map" if it should be fixed
    #     odometry_msg.child_frame_id = self.base_frame # Adjust according to your frame structure
    #     odometry_msg.pose = msg.pose  # Assign the received pose

    #     # Publish the initial pose information
    #     if not self.initialized:
    #         self.switchmap_pub.publish(odometry_msg)
    #         self.initialized = True
    #     rospy.loginfo("Published initial pose to switchMap") 

    def initialpose_callback(self, msg):
        # Process initial pose message
        rospy.loginfo(f"Received initial pose: secs {msg.header.stamp.secs}, nsecs {msg.header.stamp.nsecs}")
        self.initial_pose_msg = msg

        # Convert initial pose to Odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.frame_id = "global_map" # Or "global_map" if it should be fixed
        odometry_msg.child_frame_id = self.base_frame # Adjust according to your frame structure
        odometry_msg.pose = msg.pose  # Assign the received pose

        # Publish the initial pose information
        self.switchmap_pub.publish(odometry_msg)
        rospy.loginfo("Published initial pose to switchMap") 

if __name__ == '__main__':
    try:
        pose_subscriber = MovementTracker()
        rospy.spin()  # Keep the program running until a shutdown signal is received
    except rospy.ROSInterruptException:
        pass
