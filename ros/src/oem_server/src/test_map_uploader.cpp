#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include "your_package/UploadPointCloud.srv"  // Replace 'your_package' with the name of your package

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;

bool uploadPointCloud(your_package::UploadPointCloud::Request &req,
                      your_package::UploadPointCloud::Response &res)
{
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(req.cloud, *cloud);

    Eigen::Vector4f min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);

    // Calculate the center of the bounding box
    Eigen::Vector4f center = 0.5 * (min_point + max_point);

    // Calculate new min and max for a box scaled by the factor
    Eigen::Vector4f scaled_min, scaled_max;
    for (int i = 0; i < 3; ++i) {  // Loop through x, y, z dimensions
        float half_range = (max_point[i] - min_point[i]) * scale_factor / 2;  // Half of the scaled range
        scaled_min[i] = center[i] - half_range;
        scaled_max[i] = center[i] + half_range;
    }
    scaled_min[3] = 1.0;  // For the homogeneous coordinates
    scaled_max[3] = 1.0;


    // Compute the bounding box for the cropping operation
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(global_map_ptr_);
    crop.setMin(scaled_min);  // Adjust these parameters based on the received pose
    crop.setMax(scaled_max);     // and desired cropping area
    crop.setNegative(true);  
    crop.filter(*global_map_ptr_);

    // Registration with ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(global_map);
    PointCloud::Ptr registered_cloud(new PointCloud);
    icp.align(*registered_cloud);

    if (icp.hasConverged()) {
        res.success = true;
        res.message = "Registration successful.";
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

    // Publish map for easy visual initialization
    pcd.header.frame_id = "global_map";
    pc_map_pub_.publish(pcd);
 
    ROS_INFO_STREAM("done!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_uploader");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    private_nh.param<std::string>("pcd_path", pcd_file_path, "");

    ros::ServiceServer service = nh.advertiseService("upload_point_cloud", uploadPointCloud);
    ROS_INFO("Ready to upload point clouds.");

    ros::spin();

    return 0;
}
