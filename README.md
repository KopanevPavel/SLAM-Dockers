# Description 
Dockerfiles for some SOTA SLAM algorithms (mainly Visual Inertial Odometry with SLAM capabilities). Compiled images could be found [here](https://hub.docker.com/u/kopanev).

# Available algorithms
## Kimera
Docker for [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO-ROS) - Visual Inertial Odometry with SLAM capabilities and 3D Mesh generation.

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
