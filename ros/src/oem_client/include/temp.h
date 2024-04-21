#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
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

#include <std_srvs/Empty.h>

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NdtLocalizer{
public:

    NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~NdtLocalizer();

private:
    ros::NodeHandle nh_, private_nh_;

    // Publishers
    ros::Publisher sensor_aligned_pose_pub_;
    ros::Publisher ndt_pose_pub_;
    ros::Publisher switchmap_pose_pub_;
    ros::Publisher localMap_pose_pub_;
    ros::Publisher poly_pub_;
    ros::Publisher manual_initial_pose_pub_;

    ros::Publisher exe_time_pub_;
    ros::Publisher transform_probability_pub_;
    ros::Publisher iteration_num_pub_;

    // Subscriber
    // for testing single node
    ros::Subscriber initial_pose_sub_;
    // for testing multiple nodes
    ros::Subscriber manual_initial_pose_sub_;
    ros::Subscriber map_points_sub_;
    ros::Subscriber sensor_points_sub_;
    ros::Subscriber sensor_pose_sub;

    // service
    ros::ServiceServer save_map_service_;
    bool savePath(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    // callback functions
    void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);
    void callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);
    void callback_robot_pose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg);
    void callback_global_localization(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);

    // Tools functions
    // load a pre-tested path for searching pose
    void init_params();
    bool loadPath();
    double getNearestHeight(const geometry_msgs::Pose p);

    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr,
                       const ros::Time & time_stamp);
    bool get_transform(const std::string & target_frame, 
                       const std::string & source_frame,
                       const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::PoseStamped & pose_msg);

    // ndt status
    std::map<std::string, std::string> key_value_stdmap_;

    // variables
    std::mutex ndt_map_mtx_;

    std::string map_frame_;
    std::string base_frame_;
    std::string sensor_frame_;

    std::string load_path_file_, save_path_file_;
    geometry_msgs::PolygonStamped poly;
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;
    std::vector<geometry_msgs::Pose> poses;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt_;


    bool is_ndt_published = false;
    bool odometry_initialized = false;

    // transition matrix
    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f map_to_base_matrix;
    Eigen::Matrix4f initial_pose_matrix;
    Eigen::Matrix4f pre_trans, delta_trans;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    // For deciding whether request new submap
    geometry_msgs::Pose curr_pose_, pre_pose_;

    float traversal_dist_=0.;
    float map_switch_thres_;

    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_; 
};