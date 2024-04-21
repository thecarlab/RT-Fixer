#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import tf
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyRequest
import tf2_ros
import tf2_geometry_msgs
import message_filters
from std_msgs.msg import Float64
import numpy as np
import threading

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

class MovementTracker:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ndt_movement_tracker', anonymous=True)
        
        # Get topic names from parameters
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.local_map_frame = rospy.get_param('~local_map_frame', 'local_map_frame')
        self.map_shift_thresh = rospy.get_param('~mapShiftThresh', 40)
        self.odom_refresh_thresh = rospy.get_param('~odomRefreshThresh', 40)
        self.publish_hz = rospy.get_param('~map_odometry_tf_hz', 100)
        rospy.wait_for_service('/v1/clear_local_map')

        print("Movement Tracker Parameters:")
        print("  Base Frame: {}".format(self.base_frame))
        print("  Odometry Frame: {}".format(self.local_map_frame))
        print("  Map Shift Threshold: {} units".format(self.map_shift_thresh))
        print("  Publish Frequency: {} Hz".format(self.publish_hz))


        # Set up subscribers
        self.initialpose_sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initialpose_callback)
        self.initialpose_sub2 = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialpose_callback)
        self.ndt_pose_sub = rospy.Subscriber("ndt_pose", Odometry, self.ndt_pose_callback)
        self.lidar_odom_sub = rospy.Subscriber("/v1/odometry_node/odometry", Odometry, self.lidar_odom_callback)

        self.sub1 = message_filters.Subscriber('ndt_pose', Odometry)
        self.sub2 = message_filters.Subscriber('lidar_odom_pose', Odometry)
        
        
        #self.local_map_sub = rospy.Subscriber("filtered_localMap", PointCloud2, self.local_map_callback)

        self.ts =  message_filters.TimeSynchronizer([self.sub1, self.sub2], 10)
        self.ts.registerCallback(self.measure_deviation)

        self.deviation_pub = rospy.Publisher('distance', Float64, queue_size=10)
        self.movement_pub = rospy.Publisher('movement', Float64, queue_size=10)
        
        # Set up publisher
        self.switchmap_pub = rospy.Publisher('/switchMap', Odometry, queue_size=10)

        self.lidar_odom_pub = rospy.Publisher('lidar_odom_pose', Odometry, queue_size=10)

        self.time_diff_publisher = rospy.Publisher("/time_difference", Float64, queue_size=10)
    
        self.distance_buf = []

        self.pre_pose_msg = None
        # self.trans = Odometry()

        # translation = Vector3()
        # translation.x = 0.0  # Change these values to your desired translation
        # translation.y = -2.0
        # translation.z = -1.8

        # rotation = Quaternion()
        # rotation.x = 0.0
        # rotation.y = 0.0
        # rotation.z = 0.7071
        # rotation.w = 0.7071

        self.base_link_odom = Odometry()
        self.base_link_odom.header.frame_id = "v1/local_sensor"
        # self.base_link_odom.pose.pose.position.x = -2.0
        # self.base_link_odom.pose.pose.position.y = 0.0
        # self.base_link_odom.pose.pose.position.z = -1.8

        # self.base_link_odom.pose.pose.orientation.x = 0.0
        # self.base_link_odom.pose.pose.orientation.y = 0.0
        # self.base_link_odom.pose.pose.orientation.z = 0.0
        # self.base_link_odom.pose.pose.orientation.w = 1.0
        

        self.base_link_odom.pose.pose.position.x = rospy.get_param('~sensor_base_x', -2.0)
        self.base_link_odom.pose.pose.position.y = rospy.get_param('~sensor_base_y', 0.0)
        self.base_link_odom.pose.pose.position.z = rospy.get_param('~sensor_base_z', -1.8)

        self.base_link_odom.pose.pose.orientation.x = rospy.get_param('~sensor_base_qx', 0.0)
        self.base_link_odom.pose.pose.orientation.y = rospy.get_param('~sensor_base_qy', 0.0)
        self.base_link_odom.pose.pose.orientation.z = rospy.get_param('~sensor_base_qz', 0.0)
        self.base_link_odom.pose.pose.orientation.w = rospy.get_param('~sensor_base_qw', 1.0)

        # self.base_link_odom.pose.pose.position.x = 0.0
        # self.base_link_odom.pose.pose.position.y = -2.0
        # self.base_link_odom.pose.pose.position.z = -1.8

        # self.base_link_odom.pose.pose.orientation.x = 0.0
        # self.base_link_odom.pose.pose.orientation.y = 0.0
        # self.base_link_odom.pose.pose.orientation.z = 0.7071
        # self.base_link_odom.pose.pose.orientation.w = 0.7071

        # rotation.z = 0.0
        #rotation.w = 1.0
        # self.trans.transform.translation = translation
        # self.trans.transform.rotation = rotation

        self.ifPublishTF = False
        self.update_local_odom = False
        self.br = tf.TransformBroadcaster()

        self.accumulated_displacement_1 = 0
        self.accumulated_displacement_2 = 0
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
         
        self.tf_publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.publish_transform)

        self.tf_reader_timer = rospy.Timer(rospy.Duration(1.0 / 100), self.read_transform)

        self.local_map = None
        self.update_local_map = False

    def call_map_merge_service(self):
        
        latest_local_map = rospy.wait_for_message("filtered_localMap", PointCloud2)


        

    def measure_deviation(self, ndt_odom, lidar_odom):
        pos1 = ndt_odom.pose.pose.position
        pos2 = lidar_odom.pose.pose.position

        # Calculate the Euclidean distance between the two positions
        # distance = ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2) ** 0.5
        distance = ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5

        # Publish the calculated distance
        rospy.loginfo(ndt_odom.header.seq)
        rospy.loginfo(lidar_odom.header.seq)
        rospy.loginfo("Calculated distance: %f", distance)
        self.distance_buf.append(distance)
        if len(self.distance_buf) > 10:
            self.distance_buf.pop(0)
            mean_distance = sum(self.distance_buf) / 10
            self.deviation_pub.publish(mean_distance)

            if mean_distance > 0.5:
                thread = threading.Thread(target=self.call_map_merge_service)

    def read_transform(self, event):

        # try:
        #     # Check if the transform is available
        #     trans2 = self.tf_buffer.lookup_transform("global_map",
        #                                             msg.child_frame_id,  
        #                                             rospy.Time(0),
        #                                             rospy.Duration(1.0))  # Wait for 1 second
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logwarn("Failed to get transform from {} to {}: {}".format(self.from_frame, self.to_frame, e))
        return None

        

    def publish_transform(self, event):
        
        if self.ifPublishTF:
            if self.update_local_odom:
                return None
            self.br.sendTransform((self.odom_pose.position.x, self.odom_pose.position.y, self.odom_pose.position.z),  # Translation vector
                     (self.odom_pose.orientation.x, self.odom_pose.orientation.y, self.odom_pose.orientation.z, self.odom_pose.orientation.w),  # Rotation quaternion
                     rospy.Time.now(),     # Timestamp
                     self.local_map_frame,        # Child frame ID
                     "global_map")       # Parent frame ID

    def initialpose_callback(self, msg):
        # Process initial pose message
        print("Received initial pose: %s", msg)

        # Convert initial pose to Odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.frame_id = "global_map" # Or "global_map" if it should be fixed
        odometry_msg.child_frame_id = self.base_frame # Adjust according to your frame structure
        odometry_msg.pose = msg.pose  # Assign the received pose

        # Publish the initial pose information
        self.switchmap_pub.publish(odometry_msg)
        rospy.loginfo("Published initial pose to switchMap") 

    def lidar_odom_callback(self, msg):

        stamp = msg.header.stamp
        
        try:
            # Check if the transform is available
            trans2 = self.tf_buffer.lookup_transform("global_map",
                                                    msg.child_frame_id,  
                                                    stamp,
                                                    rospy.Duration(1.0))  # Wait for 1 second
            
            t1 = trans2.header.stamp
            t2 = msg.header.stamp
            now = rospy.get_rostime()

            rospy.loginfo(f"***trans stamp: secs {trans2.header.stamp.secs}, nsecs {trans2.header.stamp.nsecs}")
            rospy.loginfo(f"***trans stamp: secs {msg.header.stamp.secs}, nsecs {msg.header.stamp.nsecs}")
            rospy.loginfo(f"***current time: secs {now.secs}, nsecs {now.nsecs}")

            time_diff = abs(t1.to_sec() - t2.to_sec())
            self.time_diff_publisher.publish(time_diff)
            
            # Publish the time difference
            self.time_diff_publisher.publish(time_diff)
            # Transform the pose

            transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=self.base_link_odom.header, pose=self.base_link_odom.pose.pose), trans2)
            #rospy.loginfo(transformed_pose)
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=transformed_pose.header, pose=transformed_pose.pose), self.trans)
            # rospy.loginfo(transformed_pose)
            
            # transformed_odom = transform_odometry(msg, (0, -2.0, -1.8), -90)
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(header=transformed_odom.header, pose=transformed_odom.pose.pose), trans2)

            # Create new Odometry message
            transformed_odom = Odometry()
            transformed_odom.header.stamp = msg.header.stamp
            transformed_odom.header.frame_id = "global_map"
            transformed_odom.pose.pose = transformed_pose.pose

            

            
            # Publish the transformed odometry
            # self.lidar_odom_pub.publish(transform_odometry(transformed_odom, (0, -2.0, -1.8), -90))
            self.lidar_odom_pub.publish(transformed_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to get transform from {} to {}: {}".format(self.from_frame, self.to_frame, e))

    def ndt_pose_callback(self, msg):
        # Process NDT pose message
        # rospy.loginfo("Received NDT pose: %s", msg)
        if self.pre_pose_msg  is not None:
            # Calculate time difference in seconds
            time_diff = (msg.header.stamp - self.pre_time).to_sec()
            if time_diff <= 0:
                rospy.logwarn("Non-positive time difference between messages. Skipping calculation.")
                return
                

            # Calculate displacement
            dx = msg.pose.pose.position.x - self.pre_pose_msg.pose.position.x
            dy = msg.pose.pose.position.y - self.pre_pose_msg.pose.position.y
            # dz = msg.pose.pose.position.z - self.pre_pose_msg.pose.position.z
            # displacement = math.sqrt(dx**2 + dy**2 + dz**2)
            displacement = math.sqrt(dx**2 + dy**2)

            self.accumulated_displacement_1 += displacement
            self.accumulated_displacement_2 += displacement

            if self.update_local_odom:
                self.accumulated_displacement_2 -=  self.odom_refresh_thresh
                self.odom_pose = msg.pose.pose
                self.br.sendTransform((self.odom_pose.position.x, self.odom_pose.position.y, self.odom_pose.position.z),  # Translation vector
                     (self.odom_pose.orientation.x, self.odom_pose.orientation.y, self.odom_pose.orientation.z, self.odom_pose.orientation.w),  # Rotation quaternion
                     msg.header.stamp,     # Timestamp
                     self.local_map_frame,        # Child frame ID
                     "global_map")       # Parent frame ID
                now = rospy.get_rostime()
                rospy.loginfo(f"***odom_pose update time: secs {now.secs}, nsecs {now.nsecs}")
                self.update_local_odom = False
                #self.trans.transform.translation = msg.pose.pose.position
                #self.trans.transform.rotation = msg.pose.pose.orientation
                #rospy.loginfo(self.trans)

            if self.accumulated_displacement_1 >= self.map_shift_thresh:
                odometry_msg = Odometry()
                odometry_msg.header.stamp = rospy.Time.now()
                odometry_msg.header.frame_id = "global_map" # Or "global_map" if it should be fixed
                odometry_msg.child_frame_id = self.base_frame # Adjust according to your frame structure
                odometry_msg.pose = msg.pose  # Assign the received pose
                self.switchmap_pub.publish(odometry_msg)
                rospy.loginfo("Published initial pose to switchMap") 
                self.accumulated_displacement_1 -=  self.map_shift_thresh

            if self.accumulated_displacement_2 >= self.odom_refresh_thresh:
                self.update_local_odom = True

                try:
                    clear_map = rospy.ServiceProxy('/v1/clear_local_map', Empty)  # Create a service proxy
                    resp = clear_map(EmptyRequest())  # Call the service
                    print(resp)
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call failed: %s", e)
                now = rospy.get_rostime()
                rospy.loginfo(f"***local odom update time: secs {now.secs}, nsecs {now.nsecs}")

            self.movement_pub.publish(math.log(self.accumulated_displacement_2, 10))

            # Estimate speed (displacement over time)
            speed = displacement / time_diff

            # Estimate steering (angular change in yaw)
            # Convert quaternions to Euler angles (roll, pitch, yaw)
            orientation_curr = msg.pose.pose.orientation
            orientation_prev = self.pre_pose_msg.pose.orientation
            (_, _, yaw_curr) = euler_from_quaternion([orientation_curr.x, orientation_curr.y, orientation_curr.z, orientation_curr.w])
            (_, _, yaw_prev) = euler_from_quaternion([orientation_prev.x, orientation_prev.y, orientation_prev.z, orientation_prev.w])

            # Calculate change in yaw
            dyaw = yaw_curr - yaw_prev
            # Normalize the angle between -pi and pi
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

            # Log or process your speed and displacement as needed
            # rospy.loginfo("Estimated speed: %f m/s, Displacement: %f m", speed, displacement)
            rospy.loginfo("[Estimated displacement for map switch: {:.2f}]".format(self.accumulated_displacement_1))
            rospy.loginfo("[Estimated displacement for odometry refresh: {:.2f}]".format(self.accumulated_displacement_2))
            rospy.loginfo("[{:.2f}s, {:.2f}], Estimated speed: {:.2f} m/s, Displacement: {:.2f} m, Steering: {:.2f} rad".format(time_diff, 1/ time_diff, speed, displacement, dyaw))
        else:
            rospy.loginfo("Received NDT first message.")
            self.ifPublishTF = True
            self.odom_pose = msg.pose.pose

            # self.trans.transform.translation = msg.pose.pose.position
            # self.trans.transform.rotation = msg.pose.pose.orientation
            # rospy.loginfo(self.trans)
            self.travel_dist = 0
            self.lidar_odom_pub.publish(msg)

        # Save current message for next callback invocation
        self.pre_pose_msg = msg.pose
        self.pre_time = msg.header.stamp 
            




if __name__ == '__main__':
    try:
        pose_subscriber = MovementTracker()
        rospy.spin()  # Keep the program running until a shutdown signal is received
    except rospy.ROSInterruptException:
        pass
