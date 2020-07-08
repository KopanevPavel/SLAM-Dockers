# Base container - Ubuntu Melodic
FROM ubuntu:18.04

RUN echo "Installing ROS ..."

#(Ubuntu 16.04: xenial, Ubuntu 14.04: trusty, Ubuntu 18.04: bionic)
ENV UBUNTU_VERSION=bionic

#(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo, Ubuntu 18.04: melodic)
ENV ROS_VERSION=melodic

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# NOTE: Follow the official ROS installation instructions for melodic.
RUN apt-get update -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common wget
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt-get install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes

RUN apt-get update && apt-get install python-rosdep

# Update ROS environment
RUN rosdep init
RUN rosdep update

ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# Automatically source ROS for convenience:
RUN echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc

RUN echo "Installing Prerequisities ..."
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      
      # Install libraries
      apt-get update && apt-get install -y \
      python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools \
      
      ros-$ROS_VERSION-image-geometry ros-$ROS_VERSION-pcl-ros ros-$ROS_VERSION-cv-bridge && \

      apt-get update && apt-get install -y --no-install-recommends apt-utils && \
      
      apt-get update && apt-get install -y \
      cmake build-essential unzip pkg-config autoconf \
      libboost-all-dev \
      libjpeg-dev libpng-dev libtiff-dev \
      libvtk6-dev libgtk-3-dev \
      libatlas-base-dev gfortran ssh \
      libparmetis-dev \
      python-wstool python-catkin-tools python-pip python-git python-setuptools git vim && \

      apt-get install libtbb-dev

RUN pip install requests

ENV CATKIN_WS=/root/catkin_ws

# Setup catkin workspace
RUN echo "Setting up workspace ..."
RUN bash -c 'source ~/.bashrc \
      && mkdir -p ~/catkin_ws/src \
      && cd ~/catkin_ws/ && catkin init \
      && catkin config --merge-devel \
      && catkin config --extend /opt/ros/$ROS_VERSION \
      && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Clone repo
RUN echo "Clonning repo ..."
WORKDIR $CATKIN_WS/src
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git

# Install dependencies from rosinstall file using wstool
RUN wstool init
RUN wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall

# Finally, download and update repos:
RUN wstool update

WORKDIR $CATKIN_WS/src/gtsam
RUN git fetch  
RUN git checkout develop

# Compile code
WORKDIR $CATKIN_WS
RUN echo "Compiling the code ..."
RUN bash -c 'source ~/.bashrc \
      && catkin build'

ENV PORT=8090
EXPOSE 8090