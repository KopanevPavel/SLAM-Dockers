# Description 
Dockerfiles for some SOTA SLAM algorithms (mainly Visual Inertial Odometry with SLAM capabilities). Compiled images could be found [here](https://hub.docker.com/u/kopanev).

# Available algorithms
## Kimera
Docker for [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO-ROS) - Visual Inertial Odometry with SLAM capabilities and 3D Mesh generation.

![Kimera](https://img.shields.io/docker/pulls/kopanev/kimera)

Possible commands (building, pulling, cleaning):
```sh
make help
```
Run container:
```sh
./run.sh
```

## Maplab 
Docker for [Maplab](https://github.com/ethz-asl/maplab) - An open visual-inertial mapping framework. Dockerfile was created using [this](https://github.com/sofwerx/docker-maplab).

![Maplab](https://img.shields.io/docker/pulls/kopanev/maplab)

To run, use make:
```sh
make
```
This will build and run the docker container with the maplab components.

Or you can pull compiled image from Docker Hub:
```sh
docker pull kopanev/maplab:version3
```
This version includes CLion and vim for development.

Run container this command (don't forget about command xhost + for X11 server): 
```sh
xhost +
docker run -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -h $HOSTNAME -v $HOME/.Xauthority:/home/lyonn/.Xauthority kopanev/maplab:version3
```

## VINS-Mono
Docker for [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) - An optimization-based multi-sensor state estimator.

You need to put this folder into your project (as it was done in the original repo), or modify Dockerfile (to clone repo into the container)

Run container:
```sh
make build

./run.sh LAUNCH_FILE_NAME   # ./run.sh euroc.launch
```

## VINS-Fusion
Docker for [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) - An optimization-based multi-sensor state estimator.

You need to put this folder into your project (as it was done in the original repo), or modify Dockerfile (to clone repo into the container)

Run container:
```sh
make build

# Euroc Monocualr camera + IMU
./run.sh ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# Euroc Stereo cameras + IMU with loop fusion
./run.sh -l ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml

# KITTI Odometry (Stereo)
./run.sh -k ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

# KITTI Odometry (Stereo) with loop fusion
./run.sh -kl ~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

#  KITTI GPS Fusion (Stereo + GPS)
./run.sh -kg ~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml YOUR_DATASET_FOLDER/2011_10_03_drive_0027_sync/
```

## OpenVINS
Docker for [OpenVINS](https://github.com/rpng/open_vins) VIO. Note that ROS master will run on the host PC and OpenVINS will run in the container and publish all topics to the host PC.

![OpenVINS](https://img.shields.io/docker/pulls/kopanev/openvins)

Possible commands (building, pulling, cleaning):
```sh
make help
```

To run the simulation from the examples:

```sh
./run.sh pgeneva_serial_eth.launch
```

## VIORB
Docker for [VIORB](https://github.com/jingpang/LearnVIORB) - Visual Inertial ORB SLAM based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

![VIORB](https://img.shields.io/docker/pulls/kopanev/viorb)

Possible commands (building, pulling, cleaning):
```sh
make help
```

To run the container:

```sh
./run.sh
```

Then inside the container:
```sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$CATKIN_WS/src/Examples/ROS 
roslaunch ORB_VIO testeuroc.launch
```

