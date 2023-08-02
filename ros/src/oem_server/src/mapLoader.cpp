#include "mapLoader.h"

MapLoader::MapLoader(ros::NodeHandle &nh_, ros::NodeHandle &private_nh_){

    nh = nh_;
    private_nh = private_nh_;
    // specify where to read the pcd file
    private_nh.param<std::string>("pcd_path", pcd_file_path, "");

    private_nh.param<std::string>("submap_topic", submap_topic, "submap");
    // specify topics of robot's pose to obtain submap
    private_nh.param<std::string>("robot_pose_topic", robot_pose_topic, "ndt_pose");
    // specify the tf frame of the 
    private_nh.param<std::string>("local_map_frame", local_map_frame, "local_map");

    // specify the size of submap
    private_nh.param<float>("submap_size_xy", submap_size_xy_, 100);
    private_nh.param<float>("submap_size_z", submap_size_z_, 50);

    // global map publisher
    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10, true);
    // submap publisher
    // pc_submap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(submap_topic, 10, true);

    file_list_.push_back(pcd_file_path);



    createPcd();

    ndt_pose_sub_ = nh.subscribe(robot_pose_topic, 10, &MapLoader::callbackRobotPose, this);

    // clear_map_service = nh.serviceClient<std_srvs::Empty>("clear_local_map");
}

void MapLoader::createPcd()
{
	sensor_msgs::PointCloud2 pcd, part;
	for (const std::string& path : file_list_) {
		// Following outputs are used for progress bar of Runtime Manager.
		if (pcd.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
				std::cerr << "load failed " << path << std::endl;
			}
			pcd.width += part.width;
			pcd.row_step += part.row_step;
			pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
		}
		std::cerr << "load " << path << std::endl;
		if (!ros::ok()) break;
	}
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcd, *global_map_ptr_);

    // Publish map for easy visual initialization
    pcd.header.frame_id = "global_map";
    pc_map_pub_.publish(pcd);
 
    ROS_INFO_STREAM("done!");
}

void MapLoader::callbackRobotPose(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg)
{
    std::string robot_namespace;
    std::stringstream ss(ndt_odom_msg->child_frame_id); 
    if(!getline(ss, robot_namespace, '/')){
        ROS_ERROR("input frame has no namespace. Exit");
        exit(1);
    }

    if (sub_map_publishers.find(robot_namespace) == sub_map_publishers.end()){
        ROS_INFO("robot is new to the server, adding publisher");
        std::string topic_name = "/" + robot_namespace + "/submap";
        sub_map_publishers[robot_namespace] = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 10, true);
    }
        
    

    geometry_msgs::Pose pose = ndt_odom_msg->pose.pose;
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(pose.position.x-submap_size_xy_, pose.position.y-submap_size_xy_, pose.position.z-submap_size_z_, 1.0));
    box_filter.setMax(Eigen::Vector4f(pose.position.x+submap_size_xy_, pose.position.y+submap_size_xy_, pose.position.z+submap_size_z_, 1.0));

    std::cout << "min: " << pose.position.x-submap_size_xy_ << ", " << pose.position.y-submap_size_xy_ << ", " << pose.position.z-submap_size_z_ << std::endl;
    std::cout << "max: " << pose.position.x+submap_size_xy_ << ", " << pose.position.y+submap_size_xy_ << ", " << pose.position.z+submap_size_z_ << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    box_filter.setInputCloud(global_map_ptr_);
    box_filter.filter(*submap_ptr);

    sensor_msgs::PointCloud2 submap_msg;
    
    pcl::toROSMsg(*submap_ptr, submap_msg);
    
    if (submap_msg.width != 0) {
        submap_msg.header.stamp =  ros::Time::now();
        submap_msg.header.frame_id = "global_map";
        sub_map_publishers[robot_namespace].publish(submap_msg);

        ROS_INFO("new submap is published!");
    }else{
        ROS_INFO("no map points in current area");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;


    MapLoader map_loader(nh, private_nh);

    ros::spin();

    return 0;
}