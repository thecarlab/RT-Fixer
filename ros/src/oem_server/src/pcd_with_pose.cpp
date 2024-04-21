#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() : tfListener(tfBuffer), cloud_sub(nh, "/v1/filtered_points", 1), pose_sub(nh, "/v1/odometry_node/local_pose", 1) {
       
        sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>(cloud_sub, pose_sub, 10));
        sync->registerCallback(boost::bind(&PointCloudTransformer::pointCloudCallback, this, _1, _2));
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, 
                            const geometry_msgs::PoseStampedConstPtr& pose_msg) {
        sensor_msgs::PointCloud2 transformed_cloud;

        // Transform point cloud
        try {
            geometry_msgs::TransformStamped transform_stamped;
            //transform_stamped = tfBuffer.lookupTransform("target_frame", cloud_msg->header.frame_id, ros::Time(0));

            // Copy the header
            transform_stamped.header.stamp = pose_msg->header.stamp;
            transform_stamped.header.frame_id = "map";

            // Set the child_frame_id if you have one
            transform_stamped.child_frame_id = pose_msg->header.frame_id;;  // Replace with your child frame id

            // Copy the position
            transform_stamped.transform.translation.x = pose_msg->pose.position.x;
            transform_stamped.transform.translation.y = pose_msg->pose.position.y;
            transform_stamped.transform.translation.z = pose_msg->pose.position.z;

            // Copy the orientation
            transform_stamped.transform.rotation = pose_msg->pose.orientation;

            ROS_INFO("Transformed translation: %f, %f, %f", transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z);
            ROS_INFO("Transformed rotation: %f, %f, %f, %f", transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);

            tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);

            // Convert to PCL data type
            pcl::PCLPointCloud2 pcl_cloud;
            pcl_conversions::toPCL(transformed_cloud, pcl_cloud);

            // Add to buffer
            pcl::PointCloud<pcl::PointXYZ> temp_cloud;
            pcl::fromPCLPointCloud2(pcl_cloud, temp_cloud);
            // Assuming cloudBuffer is a pcl::PointCloud<pcl::PointXYZ>
            cloudBuffer += temp_cloud;
            ROS_INFO("Number of points in cloudBuffer: %lu", cloudBuffer.size());

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    ~PointCloudTransformer() {
        // // Downsample the point cloud using a VoxelGrid filter
        // Create a VoxelGrid filter object for pcl::PointXYZ type
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloudBuffer.makeShared());

        // Set the voxel size (e.g., 1cm x 1cm x 1cm)
        float leaf_size = 0.1f; // 1cm
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

        // Create a point cloud for the filtered output
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

        // Apply the filter
        voxel_filter.filter(filtered_cloud);

        // Save the filtered cloud to a PCD file
        pcl::io::savePCDFileASCII("filtered_cloud.pcd", filtered_cloud);

        // Optionally, print the size of the input and output clouds
        std::cout << "Original cloud: " << cloudBuffer.size() 
                << " points, Filtered cloud: " << filtered_cloud.size() 
                << " points." << std::endl;
        }

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    pcl::PointCloud<pcl::PointXYZ> cloudBuffer;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
    boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped>> sync;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_transformer");
    PointCloudTransformer transformer;
    ros::spin();
    return 0;
}
