FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y \
 && apt-get clean

# Setup environment
WORKDIR /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && mkdir -p src

# Get moveit stuff
RUN apt-get install -y ros-$ROS_DISTRO-moveit

# Get UR5 stuff
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git ./src/Universal_Robots_ROS_Driver \
# I think this one is outdated, but idk how calibration works && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git ./src/fmauch_universal_robot \
 && git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git ./src/universal_robot \
 && apt update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && catkin_make

# For using the ur_robot_driver with a real robot you need to install the externalcontrol-1.0.urcap which can be found inside the resources folder of this driver.

# #specific hardware acceleration for Jason's poor little integrated graphics ->http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
# xhost +
# docker run \
#   --volume=/tmp/.X11-unix:/tmp/.X11-unix \
#   --device=/dev/dri:/devdri \
#   --env="DISPLAY=$DISPLAY" \
#   your_image
RUN \
  apt-get update && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

COPY docker-entrypoint.sh .

ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/docker-entrypoint.sh && roslaunch ur5_e_moveit_config demo.launch"]
# run this: $ ./gui-docker --rm -it rosdocker:latest