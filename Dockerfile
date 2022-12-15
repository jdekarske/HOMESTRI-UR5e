FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update -qq && apt-get install -y \
 python3 python3-dev python3-pip build-essential wget\
 ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-moveit-visual-tools \
 ros-melodic-gazebo-ros ros-melodic-eigen-conversions ros-melodic-object-recognition-msgs ros-melodic-roslint \
 && pip3 install \
 rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg

# Setup environment, all installed things go here
WORKDIR /catkin_ws
RUN  mkdir src

## Get Packages that everyone will need ##
##########################################

# Use a newer Gazebo, because 9.0 is buggy
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
 && sudo apt-get update -qq \
 && sudo apt-get install gazebo9 libgazebo9-dev -y

# Get moveit stuff
RUN apt-get update -qq && apt-get install -y \
 ros-$ROS_DISTRO-moveit

# Get UR5 stuff
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git ./src/Universal_Robots_ROS_Driver
# TODO I think this is included upstream now
# RUN git clone -b calibration_devel https://github.com/fmauch/universal_robot.git ./src/fmauch_universal_robot
RUN git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git --single-branch ./src/universal_robot

# Get robotiq stuff. Unfortunately, the official one is broken :(
RUN git clone https://github.com/Polarworks/robotiq_85_gripper.git --single-branch ./src/robotiq

# A package that helps the gripper cheat
RUN git clone https://github.com/Pitrified/gazebo_ros_link_attacher.git ./src/gazebo_ros_link_attacher

# Shows the gazebo environment in rviz
RUN git clone https://github.com/andreasBihlmaier/gazebo2rviz.git ./src/gazebo2rviz\
 && git clone https://github.com/andreasBihlmaier/pysdf.git ./src/pysdf
ENV MESH_WORKSPACE_PATH='/catkin_ws/src'
ENV GAZEBO_MODEL_PATH='/catkin_ws/src'

# Get our own robot config
COPY src/ src/HOMESTRI-UR5e

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && apt-get upgrade -y \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin build

COPY docker-entrypoint.sh .
RUN echo "source /catkin_ws/docker-entrypoint.sh" >> /root/.bashrc

#########################################
# Now let's add some development stuff

# need to add a non-root user so bind mount permissions work correctly
# and dev containers work if used
ENV USERNAME ros
RUN useradd -m $USERNAME && \
  echo "$USERNAME:$USERNAME" | chpasswd && \
  usermod --shell /bin/bash $USERNAME && \
  usermod -aG sudo $USERNAME && \
  echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  # Replace 1000 with your user/group id
  usermod  --uid 1000 $USERNAME && \
  groupmod --gid 1000 $USERNAME

USER $USERNAME
RUN mkdir -p /home/$USERNAME/catkin_ws/src
WORKDIR /home/$USERNAME/catkin_ws/src
RUN echo "source /catkin_ws/docker-entrypoint.sh" >> /home/$USERNAME/.bashrc
USER root

CMD ["/bin/bash"]

