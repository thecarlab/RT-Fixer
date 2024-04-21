#include "ndt_global_localizer.h"

NdtGlobalLocalizer::NdtGlobalLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh)
{
    init_params();

    // Save path for solving height problem
    get_ndt_pose_service_ = nh_.advertiseService("get_ndt_pose", &NdtGlobalLocalizer::get_ndt_pose, this);

    // Listen to current submap
    map_points_sub_ = nh_.subscribe("/global_map", 1, &NdtGlobalLocalizer::callback_pointsmap, this);
}

NdtGlobalLocalizer::~NdtGlobalLocalizer() {}

bool NdtGlobalLocalizer::get_ndt_pose(oem_localizer::LidarGlobalLocalization::Request  &req,
         oem_localizer::LidarGlobalLocalization::Response &res)
{
    auto point_cloud_msg = req.lidar_pose.cloud; // This is a sensor_msgs::PointCloud2
    auto initial_pose_msg = req.lidar_pose.pose; // This is a geometry_msgs::PoseWithCovarianceStamped

    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    // Check if map is ready
    if (ndt_->getInputTarget() == nullptr)
    {
        ROS_WARN_STREAM_THROTTLE(1, "NO MAP for Localization!");
        return false;
    }

    initializePoseProcessing(initial_pose_msg);
    base_frame_ = point_cloud_msg.header.frame_id;
    const auto sensor_ros_time = point_cloud_msg.header.stamp;

    const auto exe_start_time = std::chrono::system_clock::now();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point_cloud_msg, *sensor_points_baselinkTF_ptr);

    // Configure NDT
    // set input point cloud
    ndt_->setInputSource(sensor_points_baselinkTF_ptr);

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

    // publish pose message
    nav_msgs::Odometry result_pose_odometry_msg;
    res.odometry.header.stamp = sensor_ros_time;
    res.odometry.header.frame_id = map_frame_;
    res.odometry.child_frame_id = base_frame_;
    res.odometry.pose.pose = result_pose_msg;

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: " << iteration_num << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
    
    return true;
}


void NdtGlobalLocalizer::init_params()
{
    private_nh_.getParam("load_path_file", load_path_file_);
    private_nh_.getParam("save_path_file", save_path_file_);
    private_nh_.getParam("map_frame", map_frame_);

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
}

void NdtGlobalLocalizer::initializePoseProcessing(const geometry_msgs::PoseWithCovarianceStamped &pose_msg) {
    // Assume all necessary checks are done before this function is called
    initial_pose_cov_msg_ = pose_msg;

    // Search for the z-axle value of the nearest history way points.
    if (load_path_file_ != "") {
        loadPath();
        initial_pose_cov_msg_.pose.pose.position.z = getNearestHeight(initial_pose_cov_msg_.pose.pose);
    }

    // Obtain transition matrix from ROS message
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
}

void NdtGlobalLocalizer::callback_pointsmap(
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

bool NdtGlobalLocalizer::loadPath()
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

double NdtGlobalLocalizer::getNearestHeight(const geometry_msgs::Pose input_init)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_localizer");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  NdtGlobalLocalizer ndt_global_localizer(nh, private_nh);

  ros::spin();

  return 0;
} 