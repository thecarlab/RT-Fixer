cmake_minimum_required(VERSION 3.0.2)
project(oem_client)

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
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

# add_executable(voxel_grid_filter src/voxel_grid_filter.cpp)

# add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})

# target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

# include_directories(include ${catkin_INCLUDE_DIRS})
# SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
# # SET(CMAKE_CXX_FLAGS "-g -Wall ${CMAKE_CXX_FLAGS}")

# add_executable(ndt_localizer_node src/ndt.cpp)
# target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})