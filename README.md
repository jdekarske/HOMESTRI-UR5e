# HOMESTRI-UR5e
[![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/jdekarske/homestri-ur5e)](https://hub.docker.com/repository/docker/jdekarske/homestri-ur5e) [![Docker Image Size (latest by date)](https://img.shields.io/docker/image-size/jdekarske/homestri-ur5e)](https://hub.docker.com/repository/docker/jdekarske/homestri-ur5e)

The UR5-e robot arm configuration for the [HOME STRI](https://homestri.ucdavis.edu/).

* [Check out the tutorial](https://github.com/jdekarske/homestri-ur5e/blob/master/ROSinDocker.md) for using ROS in docker containers! Otherwise clone as you would normally.

* See the implementation of the robot for a [remote experiment](https://github.com/jdekarske/HOMESTRI-remote-experiment)

## Quickstart
Run `docker-compose up -d` and navigate to [https://localhost:8080/vnc.html](https://localhost:8080/vnc.html) to see the robot in rviz and move it around.

### A tad more complicated
Use the `gui-docker` script in place of `docker run` like so: 
```
git clone https://github.com/jdekarske/homestri-ur5e
```
Run run gui-docker as you would for any ordinary `docker run` command. If this is your first time running this, it may take a few minutes to download the current image. This will open gui instances from the container on your host. Sometimes the windows render weird, simply resize the window and it will recover.
```
./gui-docker --rm -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-ur5e:latest
```
> "latest" is not a very stable image tag at the moment, but usable.
