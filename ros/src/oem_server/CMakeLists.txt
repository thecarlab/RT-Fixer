cmake_minimum_required(VERSION 3.0.2)
project(oem_server)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    diagnostic_msgs
    pcl_conversions
    pcl_ros
    message_generation
)

find_package(PCL REQUIRED QUIET)

add_service_files(
  FILES
  UploadPointCloud.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    diagnostic_msgs
    pcl_conversions
    pcl_ros
    message_runtime
)



catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(map_loader src/mapLoader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd_with_pose src/pcd_with_pose.cpp)
target_link_libraries(pcd_with_pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_merge src/test_map_uploader.cpp)
target_link_libraries(map_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES})