#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

class UpdateDetector
{
public:
    UpdateDetector(ros::NodeHandle& nh) :
        local_map_pose_sub(nh, "odometry_node/local_pose", 1),
        global_map_pose_sub(nh, "global_map_pose", 1),
        sync(MySyncPolicy(10), local_map_pose_sub, global_map_pose_sub),
        tf_buffer(),
        tf_listener(tf_buffer)
    {

        local_map_sub = nh.subscribe<sensor_msgs::PointCloud2>("odometry_node/local_map", 1, &UpdateDetector::localMapCallback, this);

        
        sync.registerCallback(boost::bind(&UpdateDetector::poseCallback, this, _1, _2));
        clear_map_service = nh.serviceClient<std_srvs::Empty>("clear_local_map");
        update_pub = nh.advertise<sensor_msgs::PointCloud2>("local_update", 1);
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Subscriber<geometry_msgs::PoseStamped> local_map_pose_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> global_map_pose_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;
    ros::Subscriber local_map_sub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    ros::ServiceClient clear_map_service;
    ros::Publisher update_pub;
    sensor_msgs::PointCloud2 local_map;

    void localMapCallback(const sensor_msgs::PointCloud2ConstPtr& local_map_msg)
    {
        local_map = *local_map_msg;
    }


    void poseCallback(const geometry_msgs::PoseStampedConstPtr& local_map_pose, const geometry_msgs::PoseStampedConstPtr& global_map_pose)
    {
        // Check your condition here
        // if( fulfillment condition ) {

            // Transform the local_map point cloud
            sensor_msgs::PointCloud2 global_map;
            try
            {
                tf_buffer.transform(local_map, global_map, "/global_map");
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
            }

            // Publish the transformed point cloud
            update_pub.publish(global_map);

            // Call clear map service
            std_srvs::Empty srv;
            if (!clear_map_service.call(srv))
            {
                ROS_ERROR("Failed to call service clear_local_map");
            }
        // }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_detector");
    ros::NodeHandle nh;

    UpdateDetector ud(nh);
    ros::spin();

    return 0;
}
