FROM ubuntu:16.04

# Install ROS 

#(Ubuntu 16.04: xenial, Ubuntu 14.04: trusty, Ubuntu 18.04: bionic)
ENV UBUNTU_VERSION=xenial

#(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo, Ubuntu 18.04: melodic)
ENV ROS_VERSION=kinetic

# NOTE: Follow the official ROS installation instructions for melodic.
RUN apt-get update -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y software-properties-common wget
RUN add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -
RUN apt-get update
RUN apt-get install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes

# Install framework dependencies.
# NOTE: clang-format-3.8 is not available anymore on bionic, install a newer version.
RUN apt-get install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.8 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool --yes

RUN pip install requests

# Update ROS environment
RUN rosdep init
RUN rosdep update
RUN echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc

ENV CATKIN_WS=/maplab_ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS

RUN bash -c 'source ~/.bashrc \
 && catkin init \
 && catkin config --merge-devel \
 && catkin config --extend /opt/ros/$ROS_VERSION \
 && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add gotty support for a floating maplab tty console

# Prepare golang for gotty

# gcc for cgo
RUN apt-get update && apt-get install -y --no-install-recommends \
		g++ \
		gcc \
		libc6-dev \
		make \
		pkg-config

ENV GOLANG_VERSION 1.9.4

RUN set -eux; \
	\
# this "case" statement is generated via "update.sh"
	dpkgArch="$(dpkg --print-architecture)"; \
	case "${dpkgArch##*-}" in \
		amd64) goRelArch='linux-amd64'; goRelSha256='15b0937615809f87321a457bb1265f946f9f6e736c563d6c5e0bd2c22e44f779' ;; \
		armhf) goRelArch='linux-armv6l'; goRelSha256='3c8cf3f79754a9fd6b33e2d8f930ee37d488328d460065992c72bc41c7b41a49' ;; \
		arm64) goRelArch='linux-arm64'; goRelSha256='41a71231e99ccc9989867dce2fcb697921a68ede0bd06fc288ab6c2f56be8864' ;; \
		i386) goRelArch='linux-386'; goRelSha256='d440aee90dad851630559bcee2b767b543ce7e54f45162908f3e12c3489888ab' ;; \
		ppc64el) goRelArch='linux-ppc64le'; goRelSha256='8b25484a7b4b6db81b3556319acf9993cc5c82048c7f381507018cb7c35e746b' ;; \
		s390x) goRelArch='linux-s390x'; goRelSha256='129f23b13483b1a7ccef49bc4319daf25e1b306f805780fdb5526142985edb68' ;; \
		*) goRelArch='src'; goRelSha256='0573a8df33168977185aa44173305e5a0450f55213600e94541604b75d46dc06'; \
			echo >&2; echo >&2 "warning: current architecture ($dpkgArch) does not have a corresponding Go binary release; will be building from source"; echo >&2 ;; \
	esac; \
	\
	url="https://golang.org/dl/go${GOLANG_VERSION}.${goRelArch}.tar.gz"; \
	wget -O go.tgz "$url"; \
	echo "${goRelSha256} *go.tgz" | sha256sum -c -; \
	tar -C /usr/local -xzf go.tgz; \
	rm go.tgz; \
	\
	if [ "$goRelArch" = 'src' ]; then \
		echo >&2; \
		echo >&2 'error: UNIMPLEMENTED'; \
		echo >&2 'TODO install golang-any from jessie-backports for GOROOT_BOOTSTRAP (and uninstall after build)'; \
		echo >&2; \
		exit 1; \
	fi; \
	\
	export PATH="/usr/local/go/bin:$PATH"; \
	go version

ENV GOPATH /go
ENV PATH $GOPATH/bin:/usr/local/go/bin:$PATH

RUN mkdir -p "$GOPATH/src" "$GOPATH/bin" && chmod -R 777 "$GOPATH"
WORKDIR $GOPATH

# Build gotty
RUN go get github.com/yudai/gotty

# Add support for floating tty sesssions, and some nicities for looking at how things are running
RUN apt-get install -y tmux screen curl supervisor procps net-tools bsdutils bash

WORKDIR $CATKIN_WS/src

RUN git clone https://github.com/ethz-asl/maplab.git --recursive
RUN git clone https://github.com/ethz-asl/maplab_dependencies --recursive

## Setup the linter, only needed if you are contributing back to maplab
# cd $CATKIN_WS/src/maplab
# ./tools/linter/init-git-hooks.py

WORKDIR $CATKIN_WS
RUN bash -c 'source ~/.bashrc \
 && catkin build maplab'

# Fix Poco
RUN add-apt-repository ppa:gezakovacs/poco
RUN apt-get update
RUN apt-get dist-upgrade -y

ADD run.sh /run.sh

ENV PORT=8090

EXPOSE 8090

CMD /run.sh
