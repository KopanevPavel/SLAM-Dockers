# Description 
Dockerfiles for some SOTA SLAM algorithms (mainly Visual Inertial Odometry with SLAM capabilities). Compiled images could be found [here](https://hub.docker.com/u/kopanev).

The repository is part of the publication which is under consideration in the Autonomous Robots journal. 

Sharafutdinov, D., Griguletskii, M., Kopanev, P., Kurenkov, M., Ferrer, G., Burkov, A., Gonnochenko, A., & Tsetserukou, D. (2021). **Comparison of modern open-source visual SLAM approaches**. arXiv preprint arXiv:2108.01654. [PDF](https://arxiv.org/pdf/2108.01654.pdf).

If you use this repo in academic work, please cite:


    @article{sharafutdinov2021comparison,
      title={Comparison of modern open-source visual SLAM approaches},
      author={Sharafutdinov, Dinar and Griguletskii, Mark and Kopanev, Pavel and Kurenkov, Mikhail 
              and Ferrer, Gonzalo and Burkov, Aleksey and Gonnochenko, Aleksei and Tsetserukou, Dzmitry},
      journal={arXiv preprint arXiv:2108.01654},
      year={2021}
    }

# Available algorithms
*PS some dockerfiles fail to compile automatically due to the big size or building is in progress*
## Kimera
Docker for [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO-ROS) - Visual Inertial Odometry with SLAM capabilities and 3D Mesh generation.

![](https://img.shields.io/docker/pulls/kopanev/kimera)
![](https://img.shields.io/docker/cloud/automated/kopanev/kimera)
![](https://img.shields.io/docker/cloud/build/kopanev/kimera)

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

![](https://img.shields.io/docker/pulls/kopanev/maplab)
![](https://img.shields.io/docker/cloud/automated/kopanev/maplab)
![](https://img.shields.io/docker/cloud/build/kopanev/maplab)

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

Run container using this command: 
```sh
xhost +
docker run -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -h $HOSTNAME -v $HOME/.Xauthority:/home/lyonn/.Xauthority kopanev/maplab:version3
```

## VINS-Mono
Docker for [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) - A Robust and Versatile Monocular Visual-Inertial State Estimator.

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
Docker for [OpenVINS](https://github.com/rpng/open_vins) - An open source platform for visual-inertial navigation research. Note that this container launches with flag --net=host (gives the container full access to local system services such as D-bus and is therefore considered insecure). ROS master will run on the host PC and will see all topics published in the container.

![](https://img.shields.io/docker/pulls/kopanev/openvins)
![](https://img.shields.io/docker/cloud/automated/kopanev/openvins)
![](https://img.shields.io/docker/cloud/build/kopanev/openvins)

Possible commands (building, pulling, cleaning):
```sh
make help
```

To run the simulation from the examples:

```sh
./run.sh pgeneva_serial_eth.launch
```

## VIORB
Docker for [VIORB](https://github.com/jingpang/LearnVIORB) - Visual Inertial ORB SLAM based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

![](https://img.shields.io/docker/pulls/kopanev/viorb)
![](https://img.shields.io/docker/cloud/automated/kopanev/viorb)
![](https://img.shields.io/docker/cloud/build/kopanev/viorb)

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

## OpenVSLAM

Possible commands (building, pulling, cleaning):
```sh
make help
```

To run the container:
```sh
xhost +local
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro ros:openvslam
```

## ORB_SLAM2

Pull docker [image](https://hub.docker.com/r/youyu/orb_slam2/tags?page=1&ordering=last_updated): 
```sh
docker pull youyu/orb_slam2:ubuntu18
```
([this](https://github.com/yuyou/ORB_SLAM2) fork is used)

To run the container:
```sh
xhost +local:
sudo docker run --name orb -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
youyu/orb_slam2:ubuntu18
```
