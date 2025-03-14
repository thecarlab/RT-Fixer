# 

```
sudo apt-get update
sudo apt-get install libpcap-dev
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
```


# Build

1. Install python catkin tools
```
suod apt-get update & sudo apt-get install python3-catkin-tools
```

2. Go to folder OpenEdgeMap/ros/src

```
caktin_init_workspace
```

3. Go to OpenEdgeMap/ros

```
catkin build
```

# Message Frame Definition

global_map:
    The parent frame of the whole TF tree.

vehicle/local_map:

# Test Step

1. start a roscore
```
roscore
```
2. start map server
```
roslaunch oem_server map_loader.launch namespace:=v1
```
3. start clients simulation
```
roslaunch oem_client clients.launch
rosbag play --clock -p v1 velodyne16_2019-05-23-09-06-57.bag --pause
```

command = ['roslaunch', 'oem_simulator', 'vlp16_origin.launch']
process = subprocess.Popen(command)
```
rosrun oem_client smart_rosbag.py _bag_file:=0_180.bag _clock:=true _pause:=false _prefix:=v1

rosrun oem_client smart_rosbag.py _bag_file:=kitti_2011_10_03_drive_0027_synced.bag _clock:=true _pause:=false _prefix:=v1 _rate:=0.3
```
