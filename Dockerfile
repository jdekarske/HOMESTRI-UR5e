FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update -qq && apt-get install -y \
 python3 python3-dev python3-pip build-essential ros-melodic-ros-control ros-melodic-ros-controllers \
 && pip3 install \
 rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg

# Setup environment
WORKDIR /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && mkdir src

## Get Packages that everyone will need ##
##########################################

# Get moveit stuff
RUN apt-get update -qq && apt-get install -y \
 ros-$ROS_DISTRO-moveit

# Get ROSPlan stuff
RUN apt-get update -qq && apt-get install -y \
 flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet \
 && git clone https://github.com/KCL-Planning/rosplan ./src/rosplan

# Mongodb (ROSPlan dep) stuff
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 68818C72E52529D4 \
 && echo "deb http://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.0 multiverse" >> /etc/apt/sources.list.d/mongodb-org-4.0.list \
 && apt-get update -qq && apt-get install -y \
 mongodb-org \
 && git clone -b melodic-devel https://github.com/strands-project/mongodb_store.git --single-branch ./src/mongodb_store

# Get UR5 stuff
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git --single-branch ./src/Universal_Robots_ROS_Driver \
# For using the ur_robot_driver with a real robot you need to install the externalcontrol-1.0.urcap which can be found inside the resources folder of this driver. \
# I think this one is outdated, but idk how calibration works && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git ./src/fmauch_universal_robot \
 && git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git --single-branch ./src/universal_robot

# Get robotiq stuff
RUN git clone -b kinetic-devel https://github.com/ros-industrial/robotiq.git --single-branch ./src/robotiq

# Get our own robot config
COPY homestri_robot ./src/

##########################################

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

COPY docker-entrypoint.sh .
RUN echo "source /catkin_ws/docker-entrypoint.sh" >> /root/.bashrc

CMD ["/bin/bash"]
# ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/docker-entrypoint.sh && roslaunch moveit_config demo.launch"]
# run this from the git repo: $ ./gui-docker --rm -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-ur5e:rosplan