#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/crop_box.h>
// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <vector>
// #include <pcl_ros/transforms.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <unordered_map>

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/registration/icp.h>
// #include <pcl/common/common.h>
// #include "oem_server/UploadPointCloud.h"  // Replace 'your_package' with the name of your package
// #include <pcl_ros/transforms.h>  // Make sure to include this for transformation utilities
// #include <tf/transform_listener.h> 
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/radius_outlier_removal.h>

// Standard libraries
#include <vector>
#include <unordered_map>

// PCL (Point Cloud Library) headers for handling point cloud data
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

// ROS (Robot Operating System) headers for handling ROS functionalities
#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp> 

// PCL-ROS utilities for working with ROS and PCL together
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// TF headers for handling transformations
#include <tf/transform_listener.h>


// Custom service header (make sure to replace 'oem_server' with the correct package name)
#include "oem_server/UploadPointCloud.h"

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <unordered_map>
#include <Eigen/Dense>
#include <cmath>

// Custom hash function for Eigen::Vector3i
namespace hash_eigen {
    template <typename T>
    struct hash;

    template <>
    struct hash<Eigen::Vector3i> {
        std::size_t operator()(const Eigen::Vector3i& vec) const {
            std::size_t seed = 0;
            for (int i = 0; i < vec.size(); ++i) {
                seed ^= std::hash<int>()(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
}

struct Vector3iComparator {
    bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
        if (a.x() < b.x()) return true;
        if (a.x() > b.x()) return false;
        if (a.y() < b.y()) return true;
        if (a.y() > b.y()) return false;
        return a.z() < b.z();
    }
};

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef std::unordered_map<Eigen::Vector3i, std::vector<PointT>, hash_eigen::hash<Eigen::Vector3i>> VoxelMap;


class MapLoader{
public:

    float tranversal_dist=0;
    float map_switch_thres=10.;

    MapLoader(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    bool uploadPointCloud(oem_server::UploadPointCloud::Request &req,
                     oem_server::UploadPointCloud::Response &res);
    void savePcd();

private:
    // parameters
    std::string pcd_file_path;
    std::string robot_pose_topic;
    std::string submap_topic;
    std::string local_map_frame;

    float hash_voxel_size_;
    float submap_size_xy_, submap_size_z_;

    VoxelMap voxel_global_map;
    // key variables
    ros::NodeHandle nh, private_nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;

    std::vector<std::string> file_list_;

    ros::Publisher raw_pc_map_pub_;
    ros::Publisher pc_map_pub_;

    ros::Subscriber ndt_pose_sub_;
    std::unordered_map<std::string, ros::Publisher> sub_map_publishers;


    // functions
    void createPcd();

    void callbackRobotPose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg);


    // Add for map uploading
    ros::Publisher transformed_cloud_pub;
    ros::Publisher filtered_in_cloud_pub;
    ros::Publisher filtered_out_cloud_pub;
    ros::Publisher registered_cloud_pub;

    float crop_size;
    float radius;
    float points_number;

    ros::ServiceServer service_;

};


#endif