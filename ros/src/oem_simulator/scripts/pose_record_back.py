#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
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
        self.ndt_pose_topic = rospy.get_param('~ndt_pose', '/v1/ndt_pose')
        self.local_pose_topic = rospy.get_param('~local_pose', '/v1/odometry_node/local_pose')
        self.transformed_odom_topic = rospy.get_param('~transformed_odom_topic', '/v1/ndt_odom')
        self.transformed_local_pose_topic = rospy.get_param('~transformed_local_pose_topic', '/v1/local_pose_odom')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.ndt_pose_sub = rospy.Subscriber(self.ndt_pose_topic, Odometry, lambda msg: self.odom_callback(msg, self.ndt_pose_topic))
        self.local_pose_sub = rospy.Subscriber(self.local_pose_topic, Odometry, lambda msg: self.odom_callback(msg, self.local_pose_topic))
        self.pose_pub_dict = {}
        self.pose_pub_dict[self.ndt_pose_topic] = rospy.Publisher(self.transformed_odom_topic, PoseStamped, queue_size=10)
        self.pose_pub_dict[self.local_pose_topic] = rospy.Publisher(self.transformed_local_pose_topic, PoseStamped, queue_size=10)
        


    def odom_callback(self, odom_msg, topic_name):
        
        print(topic_name)
        try:
             # Convert orientation to roll, pitch, and yaw
            roll, pitch, yaw = convert_quaternion_to_euler(odom_msg.pose.pose.orientation)
            print("x" , odom_msg.pose.pose.position.x)
            print("y" , odom_msg.pose.pose.position.y)
            print("z" , odom_msg.pose.pose.position.z)
            print("Roll:", roll)
            print("Pitch:", pitch)
            print("Yaw:", yaw)
            if not hasattr(self, 'transform'):
                self.transform = tf2_ros.TransformStamped()
                self.transform.header = odom_msg.header
                self.transform.child_frame_id = topic_name
                self.transform.transform.translation = odom_msg.pose.pose.position
                self.transform.transform.rotation = odom_msg.pose.pose.orientation
                self.inverse_transform = invert_transform(self.transform)
                #print(self.transform)
                #print(self.inverse_transform)

            transformed_pose = tf2_geometry_msgs.do_transform_pose(odom_msg.pose, self.inverse_transform)
            # transformed_pose = tf2_geometry_msgs.do_transform_pose(inverse_transform, self.transform)
            
            transformed_odom_msg = PoseStamped()
            transformed_odom_msg.header = odom_msg.header
            transformed_odom_msg.header.frame_id = topic_name
            transformed_odom_msg.pose = transformed_pose.pose
            print(odom_msg.header)
            print(transformed_odom_msg.header)

           
                
            self.pose_pub_dict[topic_name].publish(transformed_odom_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('Failed to transform odometry message: {}'.format(str(e)))
            

if __name__ == '__main__':
    node = OdometryTransformerNode()
    rospy.spin()
