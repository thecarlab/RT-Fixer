#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include "oem_server/UploadPointCloud.h"  // Replace 'your_package' with the name of your package
#include <pcl_ros/transforms.h>  // Make sure to include this for transformation utilities
#include <tf/transform_listener.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;
ros::Publisher transformed_cloud_pub;
ros::Publisher filtered_in_cloud_pub;
ros::Publisher filtered_out_cloud_pub;
ros::Publisher registered_cloud_pub;
float scale_factor;
std::vector<std::string> file_list_;

bool uploadPointCloud(oem_server::UploadPointCloud::Request &req,
                      oem_server::UploadPointCloud::Response &res)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(req.cloud, *cloud);

    // Convert geometry_msgs::Pose to Eigen::Matrix4f
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    tf::Transform pose;
    tf::poseMsgToTF(req.pose, pose);
    pcl_ros::transformAsMatrix(pose, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    Eigen::Vector4f min_point, max_point;
    pcl::getMinMax3D(*transformed_cloud, min_point, max_point);

    // Calculate the center of the bounding box
    Eigen::Vector4f center = 0.5 * (min_point + max_point);

    

    std::cout << "Min Point: " << min_point.transpose() << std::endl;
    std::cout << "Max Point: " << max_point.transpose() << std::endl;
    std::cout << "Center: " << center.transpose() << std::endl;

    sensor_msgs::PointCloud2 transformed_ros_cloud; 
    pcl::toROSMsg(*transformed_cloud, transformed_ros_cloud);
    transformed_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    transformed_ros_cloud.header.stamp = ros::Time::now();
    transformed_cloud_pub.publish(transformed_ros_cloud);

    // // Calculate new min and max for a box scaled by the factor
    Eigen::Vector4f scaled_min, scaled_max;
    for (int i = 0; i < 3; ++i) {  // Loop through x, y, z dimensions
        float half_range = (max_point[i] - min_point[i]) * scale_factor / 2;  // Half of the scaled range
        scaled_min[i] = center[i] - half_range;
        scaled_max[i] = center[i] + half_range;
    }
    scaled_min[3] = 1.0;  // For the homogeneous coordinates
    scaled_max[3] = 1.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_global_map_ptr_;
    cropped_global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_out_global_map_ptr_;
    cropped_out_global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    // // Compute the bounding box for the croppgfvfging operation
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(global_map_ptr_);
    crop.setMin(scaled_min);  // Adjust these parameters based on the received pose
    crop.setMax(scaled_max);     // and desired cropping area
    crop.setNegative(false);  
    crop.filter(*cropped_global_map_ptr_);

    // Publish the filtered point cloud
    sensor_msgs::PointCloud2 filtered_ros_cloud;
    pcl::toROSMsg(*cropped_global_map_ptr_, filtered_ros_cloud);
    filtered_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    filtered_ros_cloud.header.stamp = ros::Time::now();
    filtered_in_cloud_pub.publish(filtered_ros_cloud);

    // // Compute the bounding box for the croppgfvfging operation
    pcl::CropBox<pcl::PointXYZ> crop2;
    crop.setInputCloud(global_map_ptr_);
    crop.setMin(scaled_min);  // Adjust these parameters based on the received pose
    crop.setMax(scaled_max);     // and desired cropping area
    crop.setNegative(true);  
    crop.filter(*cropped_out_global_map_ptr_);

    // Publish the filtered point cloud
    sensor_msgs::PointCloud2 filtered_ros_cloud2;
    pcl::toROSMsg(*cropped_out_global_map_ptr_, filtered_ros_cloud2);
    filtered_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    filtered_ros_cloud.header.stamp = ros::Time::now();
    filtered_out_cloud_pub.publish(filtered_ros_cloud2);

    // // Registration with ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformed_cloud);
    icp.setInputTarget(global_map_ptr_);
    PointCloud::Ptr registered_cloud(new PointCloud);
    icp.align(*registered_cloud);

    sensor_msgs::PointCloud2 registered_ros_cloud;
    pcl::toROSMsg(*registered_cloud, registered_ros_cloud);
    registered_ros_cloud.header.frame_id = "global_map"; // Set the appropriate frame_id
    registered_ros_cloud.header.stamp = ros::Time::now();
    registered_cloud_pub.publish(registered_ros_cloud);

    *pcl::PointCloud<pcl::PointXYZ>::Ptr merged_map = new pcl::PointCloud<pcl::PointXYZ>(*cropped_out_global_map_ptr_ + *registered_cloud);
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>(*merged_map));


    if (icp.hasConverged()) {
        res.success = true;
        res.message = "Registration successful.";
        std::cout << "ICP converged." << std::endl
              << "The score is " << icp.getFitnessScore() << std::endl;
    } else {
        res.success = false;
        res.message = "Registration failed.";
    }

    return true;
}

void createPcd()
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
 
    ROS_INFO_STREAM("done!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_uploader");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    std::string pcd_file_path;

    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/transformed_cloud", 1);
    filtered_out_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/filtered_out_cloud", 1);
    filtered_in_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/filtered_in_cloud", 1);
    registered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/registered_cloud", 1);

    private_nh.param<std::string>("pcd_path", pcd_file_path, "");
    private_nh.param<float>("scale_factor", scale_factor, 0.5f);

    file_list_.push_back(pcd_file_path);
    createPcd();
    ros::ServiceServer service = nh.advertiseService("/upload_point_cloud", uploadPointCloud);
    ROS_INFO("Ready to upload point clouds.");

    ros::spin();

    return 0;
}
