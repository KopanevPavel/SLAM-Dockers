#!/bin/bash
trap : SIGTERM SIGINT

function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 LAUNCH_FILE" >&2
  exit 1
fi

roscore &
ROSCORE_PID=$!
sleep 1

OPEN_VINS_DIR=$(abspath "..")

rviz -d ${OPEN_VINS_DIR}/docker/display.rviz &
RVIZ_PID=$!

docker run \
  -it \
  --rm \
  --net=host \
  kopanev/openvins:version2 \
  /bin/bash -c \
  "cd /root/catkin_ws/; \
  catkin config \
        --env-cache \
        --extend /opt/ros/$ROS_DISTRO \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=Release; \
     source devel/setup.bash; \
     roslaunch ov_msckf ${1}"

wait $ROSCORE_PID
wait $RVIZ_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
fi
