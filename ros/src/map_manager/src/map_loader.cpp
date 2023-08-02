#include "map_loader.h"

MapLoader::MapLoader(ros::NodeHandle &nh){

    std::string pcd_file_path, init_pose_file_path, map_topic, robot_pose_topic, init_pose_topic;

    nh.param<std::string>("pcd_path", pcd_file_path, "");
    nh.param<std::string>("map_topic", map_topic, "slam_map");
    
    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 10, true);
    
    if(pcd_file_path != ""){
        ROS_INFO_STREAM("Load map from " << pcd_file_path);
        LoadPCDFromFile(pcd_file_path);
    }
    
}

void MapLoader::LoadPCDFromFile(std::string file_path){
    sensor_msgs::PointCloud2 pcd;
    if (pcl::io::loadPCDFile(file_path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << file_path << std::endl;
		}

    pcd.header.frame_id = "map";
    pc_map_pub_.publish(pcd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle nh("~");

    MapLoader map_loader(nh);

    ros::spin();

    return 0;
}