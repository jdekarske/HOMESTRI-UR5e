FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update -qq && apt-get install -y \
 python3 python3-dev python3-pip build-essential \
 ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-moveit-visual-tools \
 ros-melodic-gazebo-ros ros-melodic-eigen-conversions ros-melodic-object-recognition-msgs ros-melodic-roslint \
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

# Get UR5 stuff
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git ./src/Universal_Robots_ROS_Driver \
 && mv src/Universal_Robots_ROS_Driver/ur_controllers src/ur_controllers \
 && rm -rf src/Universal_Robots_ROS_Driver
# clone fork of the description. This is currently necessary, until the changes are merged upstream. (allegedly required? we're at bleeding edge so this doesn't work)
# RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot.git ./src/fmauch_universal_robot
# UR5e is wayyy behind on melodic integration, so I'll build from source via https://github.com/ros-industrial/universal_robot/tree/melodic-devel TODO: change this to apt when released!
RUN git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git --single-branch ./src/universal_robot

# Get robotiq stuff. Unfortunately, the official one is broken :(
RUN git clone https://github.com/StanleyInnovation/robotiq_85_gripper.git --single-branch ./src/robotiq

# A package that helps the gripper cheat
RUN git clone https://github.com/JenniferBuehler/gazebo-pkgs.git ./src/gazebo-pkg \
 && git clone https://github.com/JenniferBuehler/general-message-pkgs.git ./src/general-message-pkgs

# Get our own robot config TODO: change this to a git clone for the same package? makes development easier
COPY homestri_robot ./src/
# COPY gazebo_fix/config.yaml /root/.ignition/fuel #this doesn't work

##########################################

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
# TODO This isn't necessarily good practice, but we can fix later 
 && apt-get upgrade -y \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

COPY docker-entrypoint.sh .
RUN echo "source /catkin_ws/docker-entrypoint.sh" >> /root/.bashrc
 #fixes gazebo REST issue(gazebo needs to run one time before this is applied)
 #&& echo "sed -i 's/fuel/robotics/g' ~/.ignition/fuel/config.yaml" >> /root/.bashrc

CMD ["/bin/bash"]
# ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/docker-entrypoint.sh && roslaunch moveit_config demo.launch"]
# run this from the git repo: $ ./gui-docker -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-ur5e:latest