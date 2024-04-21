#include "ndt.h"

NdtLocalizer::NdtLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_)
{

    key_value_stdmap_["state"] = "Initializing";
    manual_initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    // Publishers (all topics here are relative to node's name) 
    // Publish the final ndt_pose of current frome
    ndt_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_pose", 10, true);
    // Record path of past ndt_pose
    poly_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("mapping_path", 10, true);
    // Debug information of NDT
    exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
    transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
    iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);

    // Allow to set initial pose through rviz
    initial_pose_sub_ = nh_.subscribe("/initialpose", 100, &NdtLocalizer::callback_init_pose, this);
    initial_pose_sub_2 = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    init_params();

    // Save path for solving height problem
    save_map_service_ = nh_.advertiseService("save_path", &NdtLocalizer::savePath, this);

    // Listen to current submap
    map_points_sub_ = nh_.subscribe("submap", 1, &NdtLocalizer::callback_pointsmap, this);
    // Listen to input filtered frame
    sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);
}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::init_params()
{
    private_nh_.getParam("load_path_file", load_path_file_);
    private_nh_.getParam("save_path_file", save_path_file_);
    private_nh_.getParam("map_frame", map_frame_);
    // private_nh_.getParam("odom_frame", odom_frame_);
    // private_nh_.getParam("local_map_frame_", local_map_frame_);
    private_nh_.getParam("base_frame", base_frame_);
    private_nh_.getParam("sensor_frame", sensor_frame_);

    // parameters for switch map
    private_nh_.getParam("map_switch_thres", map_switch_thres_);

    // Initialize an ndt tools of pcl library
    ndt_ = new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;

    double trans_epsilon = ndt_->getTransformationEpsilon();
    double step_size = ndt_->getStepSize();
    double resolution = ndt_->getResolution();
    int max_iterations = ndt_->getMaximumIterations();

    ROS_INFO(
        "Before set, trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
        step_size, resolution, max_iterations);
    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);


    ndt_->setTransformationEpsilon(trans_epsilon);
    ndt_->setStepSize(step_size);
    ndt_->setResolution(resolution);
    ndt_->setMaximumIterations(max_iterations);

    ROS_INFO(
        "After set, trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
        step_size, resolution, max_iterations);

    private_nh_.getParam("x", tf_x_);
    private_nh_.getParam("y", tf_y_);
    private_nh_.getParam("z", tf_z_);
    private_nh_.getParam("roll", tf_roll_);
    private_nh_.getParam("pitch", tf_pitch_);
    private_nh_.getParam("yaw", tf_yaw_);
    ROS_INFO_STREAM("x" << tf_x_ <<" y: "<<tf_y_<<" z: "<<tf_z_<<" roll: "
                        <<tf_roll_<<" pitch: "<< tf_pitch_<<" yaw: "<<tf_yaw_);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    // Fill in the header
    if (!ros::Time::isValid()) {
        ROS_WARN("ROS time not initialized");
    }
    pose_msg.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("The initial Pose message time: " << pose_msg.header.stamp);
    pose_msg.header.frame_id = "global_map";  // Use the correct frame ID for your setup

    tf::Quaternion q;
    q.setRPY(tf_roll_, tf_pitch_, tf_yaw_);

    // Fill in the pose data
    pose_msg.pose.pose.position.x = tf_x_; // replace with your values
    pose_msg.pose.pose.position.y = tf_y_; // replace with your values
    pose_msg.pose.pose.position.z = tf_z_; // replace with your values
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    manual_initial_pose_pub_.publish(pose_msg);
    // initializePoseProcessing(pose_msg);

    // private_nh_.getParam(
    //     "converged_param_transform_probability", converged_param_transform_probability_);

    // get TF base to sensor
    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
    get_transform(base_frame_, sensor_frame_, TF_base_to_sensor_ptr);

    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
    base_to_sensor_matrix_ = base_to_sensor_affine.matrix().cast<float>();
}

void NdtLocalizer::initializePoseProcessing(const geometry_msgs::PoseWithCovarianceStamped &pose_msg) {
    // Assume all necessary checks are done before this function is called
    initial_pose_cov_msg_ = pose_msg;

    // Search for the z-axle value of the nearest history way points.
    if (load_path_file_ != "") {
        loadPath();
        initial_pose_cov_msg_.pose.pose.position.z = getNearestHeight(initial_pose_cov_msg_.pose.pose);
        // debug_pose_marker(initial_pose_cov_msg_.pose.pose);
        poly_pub_.publish(poly);
    }

    // Obtain transition matrix from ROS message
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    // Key status variables
    is_ndt_published = false;  // Tell whether a NDT localization is done successfully for one time
}

void NdtLocalizer::callback_init_pose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
    ROS_INFO("Start to Manually initialised");

    // Ensure the pose is set in the global map frame
    if (initial_pose_msg_ptr->header.frame_id == "global_map") {
        initializePoseProcessing(*initial_pose_msg_ptr);
    } else {
        ROS_INFO("Initial pose in wrong frame");
    }
}

void NdtLocalizer::callback_pointsmap(
    const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr)
{
    ROS_INFO("Recieved new map, update old NDT registration target");
    const auto trans_epsilon = ndt_->getTransformationEpsilon();
    const auto step_size = ndt_->getStepSize();
    const auto resolution = ndt_->getResolution();
    const auto max_iterations = ndt_->getMaximumIterations();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt_new(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>);
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>* ndt_old = ndt_;

    ndt_new->setTransformationEpsilon(trans_epsilon);
    ndt_new->setStepSize(step_size);
    ndt_new->setResolution(resolution);
    ndt_new->setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new->setInputTarget(map_points_ptr);

    // swap
    //const auto swap_time = std::chrono::system_clock::now();
    ndt_map_mtx_.lock();
    ndt_ = ndt_new;
    ndt_map_mtx_.unlock();
    //const auto swap_end_time = std::chrono::system_clock::now();
    //const auto swap_use_time = std::chrono::duration_cast<std::chrono::microseconds>(swap_end_time - swap_time).count() / 1000.0;
    //std::cout << "swap map time: " << swap_use_time << std::endl;

    delete ndt_old;
}

// Designed to provide service to a specific task
void NdtLocalizer::callback_pointcloud(
    const sensor_msgs::PointCloud2::ConstPtr &sensor_points_sensorTF_msg_ptr)
{
    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    // Check if map is ready
    if (ndt_->getInputTarget() == nullptr)
    {
        ROS_WARN_STREAM_THROTTLE(1, "NO MAP for Localization!");
        return;
    }

    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

    const auto exe_start_time = std::chrono::system_clock::now();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(
      *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix_);

    // Configure NDT
    // set input point cloud
    ndt_->setInputSource(sensor_points_baselinkTF_ptr);

    // if ndt has already published, initial_pose_matrix is updated by
    // previous pose * delta pose
    if(is_ndt_published){
        initial_pose_matrix = pre_trans * delta_trans; 
    }

    // Do localization based on a guess initial pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    ndt_->align(*output_cloud, initial_pose_matrix);
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

    const Eigen::Matrix4f result_pose_matrix = ndt_->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

    const float transform_probability = ndt_->getTransformationProbability();
    const int iteration_num = ndt_->getFinalNumIteration();

    nav_msgs::Odometry result_pose_odometry_msg;
    result_pose_odometry_msg.header.stamp = sensor_ros_time;
    result_pose_odometry_msg.header.frame_id = map_frame_;
    result_pose_odometry_msg.child_frame_id = base_frame_;
    result_pose_odometry_msg.pose.pose = result_pose_msg;

    if(is_ndt_published){
        delta_trans = pre_trans.inverse() * result_pose_matrix;
        Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
        Eigen::Vector3f delta_euler = delta_trans.block<3, 1>(0, 3);
        std::cout << "delta x: " << delta_translation(0) << " y: " << delta_translation(1) << " z: " << delta_translation(2) << std::endl;

        Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
        delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
        std::cout << "delta yaw: " << delta_euler(0) << " pitch: " << delta_euler(1) << " roll: " << delta_euler(2) << std::endl;

        // define the variance of ndt localsiation
        double deviation_t, deviation_r;
        
        deviation_t = delta_translation.norm()*10;
        deviation_r = std::min(std::abs(float(M_PI)-delta_euler(0)), std::abs(delta_euler(0)))*10;
        

        // publish pose message
        
        result_pose_odometry_msg.pose.covariance[0] = deviation_t;
        result_pose_odometry_msg.pose.covariance[7] = deviation_t;
        result_pose_odometry_msg.pose.covariance[14] = deviation_t;
        result_pose_odometry_msg.pose.covariance[21] = deviation_r;
        result_pose_odometry_msg.pose.covariance[28] = deviation_r;
        result_pose_odometry_msg.pose.covariance[35] = deviation_r;
    }else{
      delta_trans =  Eigen::Matrix4f::Identity();
    }
    pre_trans = result_pose_matrix;
    Eigen::Vector3f pre_trans_translation = pre_trans.block<3, 1>(0, 3);
    std::cout << "Pre x: " << pre_trans_translation(0) << " y: " << pre_trans_translation(1) << " z: " << pre_trans_translation(2) << std::endl;


   
    ndt_pose_pub_.publish(result_pose_odometry_msg);

    if(is_ndt_published){
      geometry_msgs::PoseStamped result_pose_stamped_msg;
      result_pose_stamped_msg.header.stamp = sensor_ros_time;
      result_pose_stamped_msg.header.frame_id = map_frame_;
      result_pose_stamped_msg.pose = result_pose_odometry_msg.pose.pose;
      publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);
    }
    

    if(!is_ndt_published){
        is_ndt_published = true;
    }

    std_msgs::Float32 exe_time_msg;
    exe_time_msg.data = exe_time;
    exe_time_pub_.publish(exe_time_msg);

    std_msgs::Float32 transform_probability_msg;
    transform_probability_msg.data = transform_probability;
    transform_probability_pub_.publish(transform_probability_msg);

    std_msgs::Float32 iteration_num_msg;
    iteration_num_msg.data = iteration_num;
    iteration_num_pub_.publish(iteration_num_msg);

    // geometry_msgs::Point32 p;
    // p.x = result_pose_msg.position.x;
    // p.y = result_pose_msg.position.y;
    // p.z = result_pose_msg.position.z;

    poses.push_back(result_pose_msg);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: " << iteration_num << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
}

bool NdtLocalizer::loadPath()
{
    std::cout << "Opening path at: " << load_path_file_ << std::endl;
    std::ifstream csv;
    std::istringstream lineStream;
    csv.open(load_path_file_);
    if (csv.eof() || !csv)
    {
      ROS_ERROR("The file at path %s doesn't exist", load_path_file_.c_str());
      exit(1);
    }

    std::string line;
    std::string cell;
    int row = 0;

    geometry_msgs::PoseStamped odom;
    while (std::getline(csv, line))
    {
      lineStream.clear();
      lineStream.str(line);
      std::vector<std::string> str_odom;
      
      while (std::getline(lineStream, cell, ','))
      {
        // std::cout << cell << ",";
        str_odom.push_back(cell);
        
      }

      geometry_msgs::Point32 p;
      p.x = std::stof(str_odom[0]);
      p.y = std::stof(str_odom[1]);
      p.z = std::stof(str_odom[2]);

      poly.polygon.points.push_back(p);
    }

    poly.header.frame_id = "global_map";
    return poly.polygon.points.empty();
}

double NdtLocalizer::getNearestHeight(const geometry_msgs::Pose input_init)
{
    double z = 0, dist = std::numeric_limits<double>::max();
    for (auto pose : poly.polygon.points)
    {
      double dx = pose.x - input_init.position.x;
      double dy = pose.y - input_init.position.y;
      double ddist = sqrt(dx * dx + dy * dy);

      if (ddist < dist)
      {
        z = pose.z;
        dist = ddist;
      }
    }
    return z;
}

bool NdtLocalizer::savePath(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
    std::ofstream csv;
    csv.open(save_path_file_);
    if (!csv)
    {
        ROS_ERROR("Unable to open file at path %s", save_path_file_.c_str());
        return false;
    }

    for (const auto& pose : poses)
    {
        tf::Quaternion q(pose.orientation.x, 
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        csv << pose.position.x << ","
            << pose.position.y << ","
            << pose.position.z << ","
            << roll << ","
            << pitch << ","
            << yaw << "\n";
    }

    csv.close();
    return true;
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

void NdtLocalizer::publish_tf(
    const std::string &frame_id, const std::string &child_frame_id,
    const geometry_msgs::PoseStamped &pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_localizer");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  NdtLocalizer ndt_localizer(nh, private_nh);

  ros::spin();

  return 0;
} 