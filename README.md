# RT-Fixer
An unified HD map generating and updating platform

# ORB_SLAM2_ROS

Project originally uploaded by appliedAI-Initiative which supplies great enhancement to original ORBSLAM2 project by Raul Mur-Artal, etc. for ros integration.
We made some modification into this project in order to better understand performance of orb_slam_2.

Recommend using docker file in the folder to install the system,
the melodic version has been tested.

```
cd orb_slam_2_ros/docker/melodic
sudo docker build -t 'desired image name' .
```

# Run ORB SLAM2 ROS

first start docker container
```
sh run.sh
```

After enter the container, d435 can be replaced with l515, mono can be replaced with rgbd
```
roslaunch orb_slam_2_ros orb_slam2_d435_mono.launch
```
