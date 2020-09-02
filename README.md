# HOMESTRI-UR5e
![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/jdekarske/homestri-ur5e) ![Docker Image Size (latest by date)](https://img.shields.io/docker/image-size/jdekarske/homestri-ur5e)

My progress developing an experiment using a UR5-e robot arm for the HOME STRI.

[Check out the tutorial!](https://github.com/jdekarske/homestri-ur5e/blob/master/ROSinDocker.md)

## General instructions
Clone the repo for the gui helper script `gui-docker` (and the linked volumes if you are using them)
```
git clone https://github.com/jdekarske/homestri-ur5e
```
Run this script from the cloned the repository (ex. /home/jason/Documents/homestri-ur5e) and append arguments like you would for any ordinary `docker run` command. For example, I'm linking the volume `experimentdevel` to the container. If this is your first time running this, it may take a few minutes to download the current image. 
```
./gui-docker --rm -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-ur5e:rosplan
```
> "latest" is not a very stable image tag at the moment, but usable.

# Roadmap
- ~~Establish UCD hardware based container~~
- ~~Write tutorial~~
- Separate ROSPlan to higher-level container
- ~~Build experiment environment~~
- Write correct PDDLs
- Incorporate human inputs
- Connect actual hardware
