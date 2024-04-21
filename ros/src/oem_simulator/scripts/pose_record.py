#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
import math

def invert_transform(transform):
    """
    Invert a geometry_msgs/TransformStamped message.
    
    :param transform: The transform to be inverted (geometry_msgs/TransformStamped).
    :return: The inverted transform (geometry_msgs/TransformStamped).
    """
    # Extract the translation and rotation from the original transform
    translation = [transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z]
    rotation = [transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w]

    # Create a 4x4 matrix from the original transform
    matrix = tft.concatenate_matrices(
        tft.translation_matrix(translation),
        tft.quaternion_matrix(rotation))

    # Invert the matrix
    inverse_matrix = tft.inverse_matrix(matrix)

    # Convert the inverted matrix back to a TransformStamped message
    inverted_transform = TransformStamped()
    inverted_transform.header.frame_id = transform.child_frame_id
    inverted_transform.child_frame_id = transform.header.frame_id
    inverted_transform.transform.translation.x, inverted_transform.transform.translation.y, inverted_transform.transform.translation.z = tft.translation_from_matrix(inverse_matrix)
    inverted_transform.transform.rotation.x, inverted_transform.transform.rotation.y, inverted_transform.transform.rotation.z, inverted_transform.transform.rotation.w = tft.quaternion_from_matrix(inverse_matrix)

    return inverted_transform

def convert_quaternion_to_euler(orientation):
    """
        Convert a quaternion orientation to roll, pitch, and yaw angles.
        
    :param orientation: The quaternion orientation (geometry_msgs/Quaternion).
        :return: The roll, pitch, and yaw angles (tuple).
    """
    roll, pitch, yaw = tft.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
    return roll, pitch, yaw

class OdometryTransformerNode:
    def __init__(self):
        rospy.init_node('odometry_transformer_node')

        # Get ROS parameters for topic names
        self.odom_topic = rospy.get_param('~odom_topic', '/v1/ndt_pose')
        self.local_pose_topic = rospy.get_param('~local_pose_topic', '/v1/odometry_node/local_pose')
        self.transformed_odom_topic = rospy.get_param('~transformed_odom_topic', '/v1/ndt_odom')
        self.transformed_local_pose_topic = rospy.get_param('~transformed_local_pose_topic', '/v1/local_pose_odom')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.odom_sub = rospy.Subscriber(self.local_pose_topic, PoseStamped, self.pose_callback)
        self.ndt_odom_pub = rospy.Publisher(self.transformed_odom_topic, PoseStamped, queue_size=10)
        self.local_pose_odom_pub = rospy.Publisher(self.transformed_local_pose_topic, PoseStamped, queue_size=10)
        
    def pose_callback(self, pose_msg):
        #print(pose_msg)
        try:
             # Convert orientation to roll, pitch, and yaw
            roll, pitch, yaw = convert_quaternion_to_euler(pose_msg.pose.orientation)
            if not hasattr(self, 'transform1'):
                self.transform1 = tf2_ros.TransformStamped()
                self.transform1.header = pose_msg.header
                self.transform1.child_frame_id = 'v1/local_pose_odom' 
                self.transform1.transform.translation = pose_msg.pose.position
                self.transform1.transform.rotation = pose_msg.pose.orientation
                self.inverse_transform1 = invert_transform(self.transform1)
                #print(self.transform)
                #print(self.inverse_transform)

            temp = PoseWithCovariance()
            temp.pose = pose_msg.pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(temp, self.inverse_transform1)
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(inverse_transform, self.transform)
            
            transformed_odom_msg = PoseStamped()
            transformed_odom_msg.header = pose_msg.header
            transformed_odom_msg.header.frame_id = 'v1/local_pose_odom' 
            transformed_odom_msg.pose = transformed_pose.pose
                
            self.local_pose_odom_pub.publish(transformed_odom_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform odometry message: {}'.format(str(e)))

    def odom_callback(self, odom_msg):
        print(odom_msg)
        try:
             # Convert orientation to roll, pitch, and yaw
            roll, pitch, yaw = convert_quaternion_to_euler(odom_msg.pose.pose.orientation)
            print(roll)
            print(pitch)
            print(yaw)

            if not hasattr(self, 'transform'):
                self.transform = tf2_ros.TransformStamped()
                self.transform.header = odom_msg.header
                self.transform.child_frame_id = 'v1/ndt_odom' 
                self.transform.transform.translation = odom_msg.pose.pose.position
                self.transform.transform.rotation = odom_msg.pose.pose.orientation
                self.inverse_transform = invert_transform(self.transform)
                #print(self.transform)
                #print(self.inverse_transform)

            transformed_pose = tf2_geometry_msgs.do_transform_pose(odom_msg.pose, self.inverse_transform)
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(inverse_transform, self.transform)
            
            transformed_odom_msg = PoseStamped()
            transformed_odom_msg.header = odom_msg.header
            transformed_odom_msg.header.frame_id = 'v1/ndt_odom' 
            transformed_odom_msg.pose = transformed_pose.pose

            print(transformed_odom_msg)
            self.ndt_odom_pub.publish(transformed_odom_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform odometry message: {}'.format(str(e)))
            

if __name__ == '__main__':
    node = OdometryTransformerNode()
    rospy.spin()
