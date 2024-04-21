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

#include "oem_localizer/LidarGlobalLocalization.h"

struct Pose {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

class NdtGlobalLocalizer{
public:

    NdtGlobalLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~NdtGlobalLocalizer();

private:
    ros::NodeHandle nh_, private_nh_;

    ros::Subscriber map_points_sub_;
    // service
    ros::ServiceServer get_ndt_pose_service_;
    bool get_ndt_pose(oem_localizer::LidarGlobalLocalization::Request  &req,
         oem_localizer::LidarGlobalLocalization::Response &res);

    // callback functions
    void callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr);

    // Tools functions
    // load a pre-tested path for searching pose
    void init_params();
    bool loadPath();
    double getNearestHeight(const geometry_msgs::Pose p);
    void initializePoseProcessing(const geometry_msgs::PoseWithCovarianceStamped &pose_msg);


    // variables
    std::mutex ndt_map_mtx_;

    std::string map_frame_;
    std::string base_frame_;

    std::string load_path_file_, save_path_file_;
    geometry_msgs::PolygonStamped poly;
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;
    std::vector<geometry_msgs::Pose> poses;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> *ndt_;

    // transition matrix
    Eigen::Matrix4f initial_pose_matrix;
};