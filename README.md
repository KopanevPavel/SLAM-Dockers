# OpenVINS-docker
[![GitHub Issues Open](https://github-basic-badges.herokuapp.com/issues/KopanevPavel/OpenVINS-docker.svg)]()

Docker for [OpenVINS](https://github.com/rpng/open_vins) VIO. Note that ROS master will run on the host PC and OpenVINS will run in the container and publish all topics to the host PC.

To pull image from Docker Hub:

```
make
```

To run the simulation from the examples:

```
./run.sh pgeneva_serial_eth.launch
```

PS thx to these [guys](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) for some lines
