#include "ndt_localizer.h"

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_){

    nh_ = nh;
    private_nh_ = private_nh;
    init_params();

    timer_ = nh_.createTimer(ros::Duration(0.01), &NdtLocalizer::map_base_tf_publisher, this);

    // Publishers
    ndt_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_pose", 10);
    exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
    transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
    iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);

    // Subscribers
    initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
    map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
    sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);
}

NdtLocalizer::~NdtLocalizer(){}

void NdtLocalizer::map_base_tf_publisher(const ros::TimerEvent&){

    publish_tf(map_frame_, base_frame_, basePoseMsg);
}

void NdtLocalizer::callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr & pointcloud2_msg_ptr){
    if(!init_map){
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud2_msg_ptr, *map_points_ptr);
        ndt_->setInputTarget(map_points_ptr);
        init_map = true;
        ROS_INFO("Map is just initialized");
    }else
      ROS_INFO("Map has been initialized");
}
void NdtLocalizer::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr){
    if (pose_conv_msg_ptr->header.frame_id == map_frame_)
    {
      basePoseMsg = pose_conv_msg_ptr->pose.pose;
      preBasePoseMsg = pose_conv_msg_ptr->pose.pose;
      init_pose = true;
      ROS_INFO("Manually Update current pose");
    }else{
        ROS_INFO("Manually Localization in wrong frame");
    }
}
void NdtLocalizer::callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr){

    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

    // convert ROS msg to pcl::PointCloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);

    // check if the pose being initialized
    // if initialized, use the new initialized pose
    // else using predicted pose

    Eigen::Affine3d cur_pose_cov_msg_ = ;

    if(!init_pose){

    }

    // // get TF map to base
    // geometry_msgs::TransformStamped::Ptr TF_map_to_base_ptr(new geometry_msgs::TransformStamped);
    // get_transform(map_frame_, base_frame_, TF_map_to_base_ptr);

    return;
}

void NdtLocalizer::init_params()
{

    private_nh_.getParam("base_frame", base_frame_);
    private_nh_.getParam("map_frame", map_frame_);
    private_nh_.getParam("initial_pose_file", initial_pose_file);

    if(initial_pose_file != ""){
        std::cout << "Opening pose file at: " << initial_pose_file << std::endl;
        Eigen::Matrix4f poseMatrix;
        std::ifstream poseFile(initial_pose_file);
        if (poseFile.is_open()){
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    poseFile >> poseMatrix(i, j);
              }
            }
            poseFile.close();
        }
        else{
          ROS_ERROR("The file at path %s doesn't exist", initial_pose_file.c_str());
          exit(1);
        }

        // Extract translation
        Eigen::Vector3f translation = poseMatrix.block<3,1>(0,3);
        // Extract rotation matrix
        Eigen::Matrix3f rotationMatrix = poseMatrix.block<3,3>(0,0);
        // Convert to quaternion
        Eigen::Quaternionf quaternion(rotationMatrix);
        // Create ROS Pose message
        
        basePoseMsg.position.x = translation(0);
        basePoseMsg.position.y = translation(1);
        basePoseMsg.position.z = translation(2);
        basePoseMsg.orientation.x = quaternion.x();
        basePoseMsg.orientation.y = quaternion.y();
        basePoseMsg.orientation.z = quaternion.z();
        basePoseMsg.orientation.w = quaternion.w();

    }else{
        geometry_msgs::Pose poseMsg;
        basePoseMsg.position.x = 0.0;
        basePoseMsg.position.y = 0.0;
        basePoseMsg.position.z = 0.0;
        basePoseMsg.orientation.x = 0.0;
        basePoseMsg.orientation.y = 0.0;
        basePoseMsg.orientation.z = 0.0;
        basePoseMsg.orientation.w = 1.0;
    }

    ndt_ = new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

    double trans_epsilon = ndt_->getTransformationEpsilon();
    double step_size = ndt_->getStepSize();
    double resolution = ndt_->getResolution();
    int max_iterations = ndt_->getMaximumIterations();

    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);

    ndt_->setTransformationEpsilon(trans_epsilon);
    ndt_->setStepSize(step_size);
    ndt_->setResolution(resolution);
    ndt_->setMaximumIterations(max_iterations);

    ROS_INFO(
        "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
        step_size, resolution, max_iterations);

    // private_nh_.getParam(
    //     "converged_param_transform_probability", converged_param_transform_probability_);
}



void NdtLocalizer::publish_tf(
    const std::string &frame_id, const std::string &child_frame_id,
    const geometry_msgs::Pose &pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = ros::Time::now();

  transform_stamped.transform.translation.x = pose_msg.position.x;
  transform_stamped.transform.translation.y = pose_msg.position.y;
  transform_stamped.transform.translation.z = pose_msg.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr =
        tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try
  {
    *transform_stamped_ptr =
        tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}



// void NdtLocalizer::getXYZRPYfromMat(const Eigen::Matrix4f mat, Pose &p)
// {
//     Eigen::Vector3f translation = mat.block<3, 1>(0, 3);

//     Eigen::Matrix3f rotation_matrix = mat.block<3, 3>(0, 0);
//     Eigen::Vector3f euler = rotation_matrix.eulerAngles(0, 1, 2);

//     p.x = translation(0);
//     p.y = translation(1);
//     p.z = translation(2);
//     p.roll = euler(0);
//     p.pitch = euler(1);
//     p.yaw = euler(2);
// }

// void NdtLocalizer::timer_diagnostic()
// {
//   ros::Rate rate(100);
//   while (ros::ok())
//   {
//     diagnostic_msgs::DiagnosticStatus diag_status_msg;
//     diag_status_msg.name = "ndt_scan_matcher";
//     diag_status_msg.hardware_id = "";

//     for (const auto &key_value : key_value_stdmap_)
//     {
//       diagnostic_msgs::KeyValue key_value_msg;
//       key_value_msg.key = key_value.first;
//       key_value_msg.value = key_value.second;
//       diag_status_msg.values.push_back(key_value_msg);
//     }

//     diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
//     diag_status_msg.message = "";
//     if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing")
//     {
//       diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
//       diag_status_msg.message += "Initializing State. ";
//     }
//     if (
//         key_value_stdmap_.count("skipping_publish_num") &&
//         std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1)
//     {
//       diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
//       diag_status_msg.message += "skipping_publish_num > 1. ";
//     }
//     if (
//         key_value_stdmap_.count("skipping_publish_num") &&
//         std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5)
//     {
//       diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
//       diag_status_msg.message += "skipping_publish_num exceed limit. ";
//     }

//     diagnostic_msgs::DiagnosticArray diag_msg;
//     diag_msg.header.stamp = ros::Time::now();
//     diag_msg.status.push_back(diag_status_msg);

//     diagnostics_pub_.publish(diag_msg);

//     rate.sleep();
//   }
// }

// void NdtLocalizer::callback_init_pose(
//     const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
// {
//   if (initial_pose_msg_ptr->header.frame_id == map_frame_)
//   {
//     initial_pose_cov_msg_ = *initial_pose_msg_ptr;

//     geometry_msgs::TransformStamped transformStamped;

//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = map_frame_;
//     transformStamped.child_frame_id = odom_frame_;
//     transformStamped.transform.translation.x = initial_pose_cov_msg_.position.x;
//     transformStamped.transform.translation.y = initial_pose_cov_msg_.position.y;
//     transformStamped.transform.translation.z = initial_pose_cov_msg_.position.z;

//     transformStamped.transform.rotation.x = initial_pose_cov_msg_.orientation.x;
//     transformStamped.transform.rotation.y = initial_pose_cov_msg_.orientation.y;
//     transformStamped.transform.rotation.z = initial_pose_cov_msg_.orientation.z;
//     transformStamped.transform.rotation.w = initial_pose_cov_msg_.orientation.w;

//     transformStamped.header.stamp = ros::Time::now();
//     tf2_broadcaster_.sendTransform(transformStamped);
//     init_pose = true;

//     ROS_INFO("Manually localized");
//   }else{
//       ROS_INFO("Manually Localization in wrong frame");
//   }
// }

// void NdtLocalizer::callback_pointsmap(
//     const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr)
// {
//     if(!init_map){
//         pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
//         ndt_->setInputTarget(map_points_ptr);
//         init_map = true;
//         ROS_INFO("Map is just initialized");
//     }else
//       ROS_INFO("Map has been initialized");
// }

// void NdtLocalizer::callback_pointcloud(
//     const sensor_msgs::PointCloud2::ConstPtr &sensor_points_sensorTF_msg_ptr){

//     if (!init_map){
//         if(!init_map)
//             ROS_INFO("Map is not initialized");
        
//         return;
//     }

//     if (init_pose){
      
//     }
// }

// bool NdtLocalizer::get_transform(
//     const std::string &target_frame, const std::string &source_frame,
//     const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
// {
//   if (target_frame == source_frame)
//   {
//     transform_stamped_ptr->header.stamp = ros::Time::now();
//     transform_stamped_ptr->header.frame_id = target_frame;
//     transform_stamped_ptr->child_frame_id = source_frame;
//     transform_stamped_ptr->transform.translation.x = 0.0;
//     transform_stamped_ptr->transform.translation.y = 0.0;
//     transform_stamped_ptr->transform.translation.z = 0.0;
//     transform_stamped_ptr->transform.rotation.x = 0.0;
//     transform_stamped_ptr->transform.rotation.y = 0.0;
//     transform_stamped_ptr->transform.rotation.z = 0.0;
//     transform_stamped_ptr->transform.rotation.w = 1.0;
//     return true;
//   }

//   try
//   {
//     *transform_stamped_ptr =
//         tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
//   }
//   catch (tf2::TransformException &ex)
//   {
//     ROS_WARN("%s", ex.what());
//     ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

//     transform_stamped_ptr->header.stamp = ros::Time::now();
//     transform_stamped_ptr->header.frame_id = target_frame;
//     transform_stamped_ptr->child_frame_id = source_frame;
//     transform_stamped_ptr->transform.translation.x = 0.0;
//     transform_stamped_ptr->transform.translation.y = 0.0;
//     transform_stamped_ptr->transform.translation.z = 0.0;
//     transform_stamped_ptr->transform.rotation.x = 0.0;
//     transform_stamped_ptr->transform.rotation.y = 0.0;
//     transform_stamped_ptr->transform.rotation.z = 0.0;
//     transform_stamped_ptr->transform.rotation.w = 1.0;
//     return false;
//   }
//   return true;
// }

// void NdtLocalizer::publish_tf(
//     const std::string &frame_id, const std::string &child_frame_id,
//     const geometry_msgs::PoseStamped &pose_msg)
// {
//   geometry_msgs::TransformStamped transform_stamped;
//   transform_stamped.header.frame_id = frame_id;
//   transform_stamped.child_frame_id = child_frame_id;
//   transform_stamped.header.stamp = pose_msg.header.stamp;

//   transform_stamped.transform.translation.x = pose_msg.pose.position.x;
//   transform_stamped.transform.translation.y = pose_msg.pose.position.y;
//   transform_stamped.transform.translation.z = pose_msg.pose.position.z;

//   tf2::Quaternion tf_quaternion;
//   tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
//   transform_stamped.transform.rotation.x = tf_quaternion.x();
//   transform_stamped.transform.rotation.y = tf_quaternion.y();
//   transform_stamped.transform.rotation.z = tf_quaternion.z();
//   transform_stamped.transform.rotation.w = tf_quaternion.w();

//   tf2_broadcaster_.sendTransform(transform_stamped);
// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();

    return 0;
}