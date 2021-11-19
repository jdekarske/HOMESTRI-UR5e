# HOMESTRI-UR5e
[![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/jdekarske/homestri-ur5e)](https://hub.docker.com/repository/docker/jdekarske/homestri-ur5e) [![Docker Image Size (latest by date)](https://img.shields.io/docker/image-size/jdekarske/homestri-ur5e)](https://hub.docker.com/repository/docker/jdekarske/homestri-ur5e)

The UR5-e robot arm configuration for the [HOME STRI](https://homestri.ucdavis.edu/).

* [Check out the tutorial](https://github.com/jdekarske/homestri-ur5e/blob/master/ROSinDocker.md) for using ROS in docker containers! Otherwise clone as you would normally.

* See the implementation of the robot for a [remote experiment](https://github.com/jdekarske/HOMESTRI-remote-experiment)

* New! I added a devcontainer.json file for use with vscode's remote containers extension. To use, download the recommended extension, click the green `><` button in the corner and choose `Reopen in container`. See notes in `devcontainer.json` for handling your specific graphics hardware situation.

## Quickstart
Run `docker-compose up -d` and navigate to [https://localhost:8080/vnc.html](https://localhost:8080/vnc.html) to see the robot in rviz and move it around.
