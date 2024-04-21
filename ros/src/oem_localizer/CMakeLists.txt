cmake_minimum_required(VERSION 3.0.2)
project(oem_localizer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        message_generation
        )

add_message_files(
        FILES
        LidarPose.msg
)

add_service_files(
        FILES
        LidarGlobalLocalization.srv
)


generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs nav_msgs
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        pcl_conversions
        pcl_ros
        message_generation 
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# add_executable(ndt_localizer src/ndt_localizer.cpp)
# add_dependencies(ndt_localizer ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ndt_localizer ${catkin_LIBRARIES})

# add_executable(voxel_grid_filter src/voxel_grid_filter.cpp)
# add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})
# target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

add_executable(voxel_grid_filter src/voxel_grid_filter.cpp)

add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

add_executable(ndt_localizer_node src/ndt.cpp)
target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_global_localizer src/ndt_global_localizer.cpp)
target_link_libraries(ndt_global_localizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})