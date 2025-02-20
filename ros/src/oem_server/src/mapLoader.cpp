#include "mapLoader.h"

// Function to compute voxel indices and group points
VoxelMap createVoxelMap(PointCloudT::Ptr cloud, float voxel_size) {
    VoxelMap voxel_map;
    Eigen::Vector3i index;
    int ix, iy, iz;
    
    for (const auto& point : cloud->points) {
        ix = std::floor(point.x / voxel_size);
        iy = std::floor(point.y / voxel_size);
        iz = std::floor(point.z / voxel_size);
        index = Eigen::Vector3i(ix, iy, iz);
        voxel_map[index].push_back(point);
    }

    return voxel_map;
}

// Function to replace voxels in Point Cloud A with Point Cloud B
// void replaceVoxels(VoxelMap& voxel_map_a, const VoxelMap& voxel_map_b) {
//     for (const auto& item : voxel_map_b) {
//         if (!item.second.empty()) {
//             voxel_map_a[item.first] = item.second;
//         }
//     }
// }


void replaceVoxels(VoxelMap& voxel_map_a, const VoxelMap& voxel_map_b) {
    std::set<Eigen::Vector3i, Vector3iComparator> already_removed; // To track removed voxels and avoid repetition

    // for (const auto& item : voxel_map_b) {
    //     if (!item.second.empty()) {
    //         // Loop over the range from -10 to 10, excluding the zero since it will be replaced
    //         for (int dz = -10; dz <= 10; dz++) {
    //             if (dz == 0) continue; // Skip the current voxel level, only remove above and below
    //             Eigen::Vector3i neighbor(item.first[0], item.first[1], item.first[2] + dz);

    //             if (voxel_map_a.find(neighbor) != voxel_map_a.end())
    //                 voxel_map_a.erase(neighbor);
    //             // Check if already removed to avoid repeat operations
    //             // if (already_removed.find(neighbor) == already_removed.end()) {
    //             //     voxel_map_a.erase(neighbor);
    //             //     already_removed.insert(neighbor); // Mark this voxel as removed
    //             // }
    //         }
    //         // Replace the voxel in A with the voxel from B
    //         // voxel_map_a[item.first] = item.second;
    //     }
    // }

    for (const auto& item : voxel_map_b) {
        if (!item.second.empty()) {
            voxel_map_a[item.first] = item.second;
        }
    }
}

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
    private_nh.param<float>("hash_voxel_size", hash_voxel_size_, 1.0f);
    private_nh.param<float>("submap_size_xy", submap_size_xy_, 100);
    private_nh.param<float>("submap_size_z", submap_size_z_, 50);

    private_nh.param<float>("crop_size", crop_size, 50.0f);
    private_nh.param<float>("filter_radius", radius, 10.0f);
    private_nh.param<float>("filter_points", points_number, 100.0f);

    // global map publisher
    raw_pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/raw_global_map", 10, true);
    pc_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10, true);
    // submap publisher
    // pc_submap_pub_ = nh.advertise<sensor_msgs::PointCloud2>(submap_topic, 10, true);

    file_list_.push_back(pcd_file_path);



    createPcd();

    ndt_pose_sub_ = nh.subscribe(robot_pose_topic, 10, &MapLoader::callbackRobotPose, this);

    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/transformed_cloud", 1, true);
    filtered_out_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/cropped_update_cloud", 1, true);
    filtered_in_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/cropped_global_cloud", 1, true);
    registered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/registered_cloud", 1, true);

    service_ = nh.advertiseService("/upload_point_cloud", &MapLoader::uploadPointCloud, this);

    

}


bool MapLoader::uploadPointCloud(oem_server::UploadPointCloud::Request &req,
                      oem_server::UploadPointCloud::Response &res)
{

    ROS_INFO("Start update global map");
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud, *transformed_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr redundancy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.redundancy_cloud, *redundancy_cloud);

    // // Convert geometry_msgs::Pose to Eigen::Matrix4f
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // tf::Transform pose;
    // tf::poseMsgToTF(req.pose, pose);
    // pcl_ros::transformAsMatrix(pose, transform);

    // // Transform point cloud from local coordinates to global_map frame
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*update_cloud, *transformed_cloud, transform);

    Eigen::Vector4f min_point, max_point;
    Eigen::Vector4f scaled_min, scaled_max;
    // pcl::getMinMax3D(*transformed_cloud, min_point, max_point);

    // Calculate the center of the bounding box
    // std::cout << "get new update" << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    // voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    // voxelgrid.setInputCloud(update_cloud);
    // voxelgrid.filter(*downsampled);
    // *transformed_cloud = *downsampled;

    // std::cout << "downsample update" << std::endl;

    // std::cout << "Min Point: " << min_point.transpose() << std::endl;
    // std::cout << "Max Point: " << max_point.transpose() << std::endl;
    // std::cout << "Center: " << center.transpose() << std::endl;

    // Outlier far from center of the update point cloud is mostly sparse, 
    // use radius outlier filter to remove them
    // Make sure min_point and max_point concentrate on the center area of the update map
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(transformed_cloud);
    // outrem.setRadiusSearch(radius);  // Set the radius of the neighborhood considered
    // outrem.setMinNeighborsInRadius(points_number);  // Set the minimum number of neighbors within this radius
    // outrem.filter(*transformed_cloud);

    // std::cout << "filter radius done" << std::endl;

    // Publish transformed and filterred map
    sensor_msgs::PointCloud2 transformed_ros_cloud; 
    pcl::toROSMsg(*transformed_cloud, transformed_ros_cloud);
    transformed_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    transformed_ros_cloud.header.stamp = ros::Time::now();
    transformed_cloud_pub.publish(transformed_ros_cloud);

    ROS_INFO("Publish debug clouds done");

    // pcl::getMinMax3D(*transformed_cloud, min_point, max_point);
    Eigen::Vector4f center = 0.5 * (min_point + max_point);

    // // Calculate new min and max for a box scaled by the factor
    
    // for (int i = 0; i < 2; ++i) {  // Loop through x, y, z dimensions
    //     float half_range = (max_point[i] - min_point[i]) * scale_factor / 2;  // Half of the scaled range
    //     scaled_min[i] = center[i] - half_range;
    //     scaled_max[i] = center[i] + half_range;
    // }

    // scaled_min[2] = min_point[2];  // For the homogeneous coordinates
    // scaled_max[2] = max_point[2];

    // scaled_min[3] = min_point[3];  // For the homogeneous coordinates
    // scaled_max[3] = max_point[3];

    // std::cout << "1st Min Point: " << min_point.transpose() << std::endl;
    // std::cout << "1st Max Point: " << max_point.transpose() << std::endl;

    // std::cout << "1st Scaled Min Point: " << scaled_min.transpose() << std::endl;
    // std::cout << "1st Scaled Max Point: " << scaled_max.transpose() << std::endl;

    // std::cout << "1st Center: " << center.transpose() << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cropped_global_map_ptr_;
    // temp_cropped_global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    //     // Compute the bounding box for the croppgfvfging operation
    // pcl::CropBox<pcl::PointXYZ> crop3;
    // crop3.setInputCloud(global_map_ptr_);
    // crop3.setMin(scaled_min);  // Adjust these parameters based on the received pose
    // crop3.setMax(scaled_max);     // and desired cropping area
    // crop3.setNegative(true);  
    // crop3.filter(*temp_cropped_global_map_ptr_);

    // // Registration with ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(transformed_cloud);
    // icp.setInputTarget(global_map_ptr_);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // icp.align(*registered_cloud);

    // Initializing Normal Distributions Transform (NDT)
    // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // ndt.setTransformationEpsilon(0.001);
    // ndt.setStepSize(0.1);
    // ndt.setResolution(3.0);
    // ndt.setMaximumIterations(100);
    // ndt.setInputSource(transformed_cloud);
    // ndt.setInputTarget(global_map_ptr_);

    // // Set initial alignment estimate found using robot odometry.
    // Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // // Calculating required rigid transform to align the input cloud to the target cloud.
    // pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ndt.align(*registered_cloud, init_guess);

    // std::cout << "ndt registration" << std::endl;
    VoxelMap redundancy_map = createVoxelMap(redundancy_cloud, hash_voxel_size_);
    int count = 0;
    for (const auto& item : redundancy_map) {
        if (voxel_global_map.find(item.first) != voxel_global_map.end() && !item.second.empty()) {
            voxel_global_map.erase(item.first);
            count += 1;
        }
    }
    std::cout << "Total unique voxels in redundancy_map: " << redundancy_map.size() << std::endl;
    std::cout << "Remove redundancy voxel " << count << std::endl;
    std::cout << "Total unique voxels in raw global map: " << voxel_global_map.size() << std::endl;

    ROS_INFO("Remove Error points");

    PointCloudT::Ptr modified_cloud_b(new PointCloudT);
    for (const auto& voxel : voxel_global_map) {
        modified_cloud_b->points.insert(modified_cloud_b->points.end(), voxel.second.begin(), voxel.second.end());
    }

    modified_cloud_b->width = modified_cloud_b->points.size();
    modified_cloud_b->height = 1;
    modified_cloud_b->is_dense = true;
    // global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>(*modified_cloud_b));

    sensor_msgs::PointCloud2 pcd2;
    pcl::toROSMsg(*modified_cloud_b, pcd2);
    // Publish map for easy visual initialization
    pcd2.header.frame_id = "global_map";
    pcd2.header.stamp =  ros::Time::now();
    filtered_out_cloud_pub.publish(pcd2);

    ROS_INFO("Remove Error points publish");

    VoxelMap voxel_local_map = createVoxelMap(transformed_cloud, hash_voxel_size_);
    // Print the number of unique voxels in the map
    std::cout << "Total unique voxels in update local map: " << voxel_local_map.size() << std::endl;
    replaceVoxels(voxel_global_map, voxel_local_map);

    std::cout << "replacement done" << std::endl;

    // pcl::getMinMax3D(*registered_cloud, min_point, max_point);
    // center = 0.5 * (min_point + max_point);

    // for (int i = 0; i < 2; ++i){
    //     scaled_min[i] = center[i] - crop_size;
    //     scaled_max[i] = center[i] + crop_size;
    // }

    // scaled_min[2] = min_point[2];
    // scaled_max[2] = max_point[2];

    // scaled_min[3] = 1.0;  // For the homogeneous coordinates
    // scaled_max[3] = 1.0;

    // Calculate new min and max for a box scaled by the factor
    
    // for (int i = 0; i < 2; ++i) {  // Loop through x, y, z dimensions
    //     float half_range = (max_point[i] - min_point[i]) * scale_factor / 2;  // Half of the scaled range
    //     scaled_min[i] = center[i] - half_range;
    //     scaled_max[i] = center[i] + half_range;
    // }

    // scaled_min[2] = min_point[2];  // For the homogeneous coordinates
    // scaled_max[2] = max_point[2];

    // scaled_min[3] = min_point[3];  // For the homogeneous coordinates
    // scaled_max[3] = max_point[3];

    // std::cout << "Min Point: " << min_point.transpose() << std::endl;
    // std::cout << "Max Point: " << max_point.transpose() << std::endl;

    // std::cout << "Scaled Min Point: " << scaled_min.transpose() << std::endl;
    // std::cout << "Scaled Max Point: " << scaled_max.transpose() << std::endl;

    // std::cout << "Center: " << center.transpose() << std::endl;
    
    // scaled_min = min_point;
    // scaled_max = max_point;
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_global_map_ptr_;
    // cropped_global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_update_map_ptr_;
    // cropped_update_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // // Compute the bounding box for the croppgfvfging operation
    // pcl::CropBox<pcl::PointXYZ> crop;
    // crop.setInputCloud(global_map_ptr_);
    // crop.setMin(scaled_min);  // Adjust these parameters based on the received pose
    // crop.setMax(scaled_max);     // and desired cropping area
    // crop.setNegative(true);  
    // crop.filter(*cropped_global_map_ptr_);



    // Publish the filtered point cloud
    // sensor_msgs::PointCloud2 filtered_ros_cloud;
    // pcl::toROSMsg(*cropped_global_map_ptr_, filtered_ros_cloud);
    // filtered_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    // filtered_ros_cloud.header.stamp = ros::Time::now();
    // filtered_in_cloud_pub.publish(filtered_ros_cloud);

    // // // Compute the bounding box for the croppgfvfging operation
    // pcl::CropBox<pcl::PointXYZ> crop2;
    // crop.setInputCloud(registered_cloud);
    // crop.setMin(scaled_min);  // Adjust these parameters based on the received pose
    // crop.setMax(scaled_max);     // and desired cropping area
    // crop.setNegative(false);  
    // crop.filter(*cropped_update_map_ptr_);

    // // Publish the filtered point cloud
    // sensor_msgs::PointCloud2 filtered_ros_cloud2;
    // pcl::toROSMsg(*cropped_update_map_ptr_, filtered_ros_cloud2);
    // filtered_ros_cloud2.header.frame_id = "global_map"; // Set the appropriate frame_id
    // filtered_ros_cloud2.header.stamp = ros::Time::now();
    // filtered_out_cloud_pub.publish(filtered_ros_cloud2);

    sensor_msgs::PointCloud2 registered_ros_cloud;
    pcl::toROSMsg(*redundancy_cloud, registered_ros_cloud);
    registered_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    registered_ros_cloud.header.stamp = ros::Time::now();
    registered_cloud_pub.publish(registered_ros_cloud);

    ROS_INFO("redundancy_cloud points publish");

    // global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cropped_global_map_ptr_ + *cropped_update_map_ptr_));

    // if (ndt.hasConverged()) {
    //     std::cout << "NDT converged." << std::endl
    //           << "The score is " << ndt.getFitnessScore() << std::endl;
    // } 

    PointCloudT::Ptr modified_cloud_a(new PointCloudT);
    for (const auto& voxel : voxel_global_map) {
        modified_cloud_a->points.insert(modified_cloud_a->points.end(), voxel.second.begin(), voxel.second.end());
    }

    modified_cloud_a->width = modified_cloud_a->points.size();
    modified_cloud_a->height = 1;
    modified_cloud_a->is_dense = true;
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>(*modified_cloud_a));

     ROS_INFO("Finish update global map");
    sensor_msgs::PointCloud2 pcd;
    pcl::toROSMsg(*global_map_ptr_, pcd);
    // Publish map for easy visual initialization
    pcd.header.frame_id = "global_map";
    pcd.header.stamp =  ros::Time::now();
    pc_map_pub_.publish(pcd);

    // global_pcd.header.stamp = ros::Time(0);
    // raw_pc_map_pub_.publish(global_pcd);

    std::cout << "Update done" << std::endl;

    return true;
}

void MapLoader::savePcd()
{
    if (global_map_ptr_) {
        // Define the filename
        std::string filename = "/mnt/ros_ws/src/oem_server/maps/update_vlp-16.pcd";
        
        // Save the point cloud to a file
        if (pcl::io::savePCDFile<pcl::PointXYZ>(filename, *global_map_ptr_) == 0) {
            std::cout << "Saved merged point cloud to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save point cloud." << std::endl;
        }
    } else {
        std::cerr << "No point cloud data to save." << std::endl;
    }
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

    voxel_global_map = createVoxelMap(global_map_ptr_, hash_voxel_size_);
    // Print the number of unique voxels in the map
    std::cout << "Total unique voxels in raw global map: " << voxel_global_map.size() << std::endl;

    // Publish map for easy visual initialization
    pcd.header.frame_id = "global_map";
    raw_pc_map_pub_.publish(pcd);
    global_pcd = pcd;
 
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
        submap_msg.header.stamp =  ros::Time::now();
        submap_msg.header.frame_id = "global_map";
        sub_map_publishers[robot_namespace].publish(submap_msg);
        ROS_INFO("no map points in current area");
    }
}

MapLoader* global_map_loader_ptr = nullptr;

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
    
    // Call kill method on map_loader
    if (global_map_loader_ptr) {
        global_map_loader_ptr->savePcd();
    }

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_loader");

    ROS_INFO("\033[1;32m---->\033[0m Map Loader Started.");

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;


    MapLoader map_loader(nh, private_nh);
    global_map_loader_ptr = &map_loader;
    signal(SIGINT, mySigintHandler);

    ros::spin();

    return 0;
}