#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class NdtLocalizer{
public:

    NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~NdtLocalizer();

private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber initial_pose_sub_;
    ros::Subscriber map_points_sub_;
    ros::Subscriber sensor_points_sub_;

    ros::Publisher ndt_pose_pub_;
    ros::Publisher exe_time_pub_;
    ros::Publisher transform_probability_pub_;
    ros::Publisher iteration_num_pub_;

    std::string base_frame_;
    std::string map_frame_;
    std::string initial_pose_file;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    geometry_msgs::Pose basePoseMsg;
    geometry_msgs::Pose preBasePoseMsg;
    // init guess for ndt
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;

    
    ros::Timer timer_;

    bool init_map = false;
    bool init_pose = false;

    // function
    void init_params();
    bool loadInitialPose(std::string pose_file);
    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
                       const ros::Time & time_stamp);
    bool get_transform(const std::string & target_frame, 
                       const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);

    void map_base_tf_publisher(const ros::TimerEvent&);

    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::Pose & pose_msg);

    void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);
    void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    

    // Eigen::Matrix4f base_to_sensor_matrix_;
    // Eigen::Matrix4f odom_trans, pre_odom_trans;
    // Eigen::Matrix4f initial_pose_matrix, map_to_odom_matrix;
    // Eigen::Matrix4f pre_trans, delta_trans, pre_corr_trans;

    // bool init_pose = false;
    // bool init_map = false;
    // bool is_ndt_published = false;

    

    // std::string path_file;
    // geometry_msgs::PolygonStamped poly;

    
    // // init guess for ndt
    // geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;

    // std::mutex ndt_map_mtx_;

    // double converged_param_transform_probability_;
    // std::thread diagnostic_thread_;
    // std::map<std::string, std::string> key_value_stdmap_;

    
    // void timer_diagnostic();

    // bool get_transform(const std::string & target_frame, const std::string & source_frame,
    //                    const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
    //                    const ros::Time & time_stamp);
    // bool get_transform(const std::string & target_frame, 
    //                    const std::string & source_frame,
    //                    const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
    // void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
    //                 const geometry_msgs::PoseStamped & pose_msg);

    // void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    // void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);
    // void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    // void callback_odom(const nav_msgs::Odometry::ConstPtr & odom_msg_ptr);

    // void getXYZRPYfromMat(const Eigen::Matrix4f mat, Pose & p);
    // double getNearestHeight(const geometry_msgs::Pose p);
    // bool loadPath(std::string path);

};// NdtLocalizer Core