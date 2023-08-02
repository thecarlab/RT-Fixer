#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <unordered_map>

class MapLoader{
public:

    float tranversal_dist=0;
    float map_switch_thres=10.;

    MapLoader(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

private:
    // parameters
    std::string pcd_file_path;
    std::string robot_pose_topic;
    std::string submap_topic;
    std::string local_map_frame;

    float submap_size_xy_, submap_size_z_;

    // key variables
    ros::NodeHandle nh, private_nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;

    std::vector<std::string> file_list_;

    ros::Publisher pc_map_pub_;

    ros::Subscriber ndt_pose_sub_;
    std::unordered_map<std::string, ros::Publisher> sub_map_publishers;


    // functions
    void createPcd();
    void callbackRobotPose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg);
};


#endif