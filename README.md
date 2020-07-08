# Description 
Dockerfiles for some SOTA SLAM algorithms (mainly Visual Inertial Odometry with SLAM capabilities). Compiled images could be found [here](https://hub.docker.com/u/kopanev)

# Available algorithms
## Kimera
Docker for [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO-ROS) - Visual Inertial Odometry with SLAM capabilities and 3D Mesh generation.

## Maplab 
Docker for [Maplab](https://github.com/ethz-asl/maplab) - An open visual-inertial mapping framework.

## VINS-Mono
Docker for [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) - An optimization-based multi-sensor state estimator

## VINS-Fusion
Docker for [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) - An optimization-based multi-sensor state estimator

## OpenVINS
Docker for [OpenVINS](https://github.com/rpng/open_vins) VIO. Note that ROS master will run on the host PC and OpenVINS will run in the container and publish all topics to the host PC.

To pull image from Docker Hub:

```
make
```

To run the simulation from the examples:

```
./run.sh pgeneva_serial_eth.launch
```

## VIORB
Docker for [VIORB](https://github.com/jingpang/LearnVIORB) - Visual Inertial ORB SLAM based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
