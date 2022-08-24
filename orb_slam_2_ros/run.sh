xhost +local:docker 
# Start a terminal
docker run -it --rm --net=host --gpus all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    -v /home/nino/.Xauthority:/root/.Xauthority \
    -v /mnt/zr/openEdgeMap/orb_slam2_ros/Downloads:/home/nvidia/Downloads \
   moetiger/orbslam2-ros-melodic:latest 
