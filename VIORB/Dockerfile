# Base container (kinetic-perception is a metapackage of ROS containing all the perception-related packages, such as OpenCV, PCL, and so on)
FROM ros:kinetic-perception

ENV CATKIN_WS=/root/catkin_ws

      # Set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \

      # Install libraries
      apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libblas-dev \
      liblapack-dev \
      python-catkin-tools \
      vim \
      libomp-dev \
	libboost-dev libboost-thread-dev libboost-filesystem-dev \
	libboost-serialization-dev \
	qt5-default \
      libglew-dev \
	ca-certificates \
      python-pip python-git \
	curl htop nano wget git unzip \
      libopencv-dev \
      build-essential pkg-config \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-libg2o \
      ros-${ROS_DISTRO}-tf && \
      rm -rf /var/lib/apt/lists/*

# ORB_SLAM2 package needs an older version of Eigen in order to compile
RUN wget "http://bitbucket.org/eigen/eigen/get/3.2.8.tar.gz"
RUN tar zxvf 3.2.8.tar.gz && cd eigen* && mkdir build && cd build && cmake .. && make && make install

# Build boost
RUN cd /tmp && wget https://dl.bintray.com/boostorg/release/1.73.0/source/boost_1_73_0.tar.bz2 \
	&& tar xjvf boost_1_73_0.tar.bz2 \
	&& cd boost_1_73_0 \
	&& bash bootstrap.sh \
	&& ./b2 && ./b2 install \
	&& cd / && rm -rf /tmp/boost_1_73_0*

ENV OpenCV_DIR /usr/share/OpenCV

# Install Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make install

# Copy VIORB
# RUN mkdir -p $CATKIN_WS/src/VIORB/
# COPY ./ $CATKIN_WS/src/VIORB/
WORKDIR $CATKIN_WS/src
# RUN git clone https://github.com/jingpang/LearnVIORB
RUN git clone https://github.com/KopanevPavel/LearnVIORB.git
RUN mv $CATKIN_WS/src/LearnVIORB/* $CATKIN_WS/src
RUN rm -r -v LearnVIORB

ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

RUN echo "Configuring and building Thirdparty/DBoW2 ..."
WORKDIR $CATKIN_WS/src/Thirdparty/DBoW2
RUN mkdir build
WORKDIR $CATKIN_WS/src/Thirdparty/DBoW2/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j

RUN echo "Configuring and building Thirdparty/g2o ..."
WORKDIR $CATKIN_WS/src/Thirdparty/g2o
RUN mkdir build
WORKDIR $CATKIN_WS/src/Thirdparty/g2o/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j

RUN echo "Uncompress vocabulary ..."
WORKDIR $CATKIN_WS/src/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz

RUN echo "Configuring and building ORB_SLAM2 ..."
WORKDIR $CATKIN_WS/src
RUN mkdir build
WORKDIR $CATKIN_WS/src/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j

ENV ROS_PACKAGE_PATH=$CATKIN_WS/src/Examples/ROS:/opt/ros/$ROS_DISTRO/share:/opt/ros/$ROS_DISTRO/stacks
# ENV ROS_ROOT=/opt/ros/$ROS_DISTRO/share/ros

RUN echo "Building ROS nodes"
WORKDIR $CATKIN_WS/src/Examples/ROS/ORB_SLAM2
RUN mkdir build
WORKDIR $CATKIN_WS/src/Examples/ROS/ORB_SLAM2/build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$CATKIN_WS/src/Examples/ROS && echo $ROS_PACKAGE_PATH && echo $ROS_ROOT && cmake .. -DROS_BUILD_TYPE=Release && make -j"

WORKDIR $CATKIN_WS/src/Examples/ROS/ORB_VIO
RUN mkdir build
WORKDIR $CATKIN_WS/src/Examples/ROS/ORB_VIO/build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$CATKIN_WS/src/Examples/ROS && echo $ROS_PACKAGE_PATH && echo $ROS_ROOT && cmake .. -DROS_BUILD_TYPE=Release && make -j"

