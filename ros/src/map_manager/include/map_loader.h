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

class MapLoader{
public:

    ros::Publisher pc_map_pub_;
    MapLoader(ros::NodeHandle &nh);

private:

    void LoadPCDFromFile(std::string file_path);

}; //MapLoader

#endif