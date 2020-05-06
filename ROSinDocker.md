> If you were directed here simply to play with robots, skip to the [quickstart]() section to skip the technical stuff. It'll take 5 minutes, we promise.

> This document is in construction while we are learning as well; these are the current strategies. Please contribute or contact us for help!

# ROS in Docker
This short tutorial explains a method for experimenting in [ROS](https://www.ros.org/) using [Docker](https://www.docker.com/) containers as a development environment. This tutorial was developed for researchers in the [Habitats Optimized for Missions of Exploration](https://homestri.ucdavis.edu/) research institute to share experimental environments and showcase our work. Here, we'll walk you through installing Docker, working with containers, working *in* containers, and most importantly, playing with robots!

We'll assume you have some basic commandline experience to get started, but working with ROS requires knowledge of navigating a linux environment which is best learned through practice. The hope is that this tutorial will allow you to get started with **robots** and learn about linux along the way, without having to struggle with setup. The added takeaway is that *your* container development environment is portable to anyone with Docker, allowing others to share in the cool work that you've done.

This tutorial will **not** go over ROS itself. Please see the wiki or other tutorials (try it in a container :smile:)

> - [ROS tutorials](http://wiki.ros.org/ROS/Tutorials)
> - [edX](https://www.edx.org/course/hello-real-world-with-ros-robot-operating-system)
> 

---

## What's a robot?kage management, enter thekage management, enter the following in your terminal to update your liskage management, enter the following in your terminal to update your lis following in your terminal to update your lis
The Robot Operating System (ROS) is an open source ecosystem of robot models, algorithms, environments, tools, and ideas. The modularity of ROS allows your projects to use packages that others have developed and maintain so that you can improve your robot by standing on the shoulders of giants. Did you buy a robot from companyx? Chances are, companyx has developed a ROS package that contains the robot configuration, controls, 3D models, and demos. Why reinvent their wheel? If you are worried about the accuracy if their models, there's a good chance they're a part of an industry [consortium](https://rosindustrial.org/) (if you find an error, please support open source development and submit an issue on github!).

If you've been introduced to ROS previously you are intimately aware of the particularities in installing linux (which distro? beaver...what?). Then, you must find the right version of ROS (what? the package you need isn't supported?!? start over!). Since ROS is composed of so many pieces, keeping track of what goes where can be a huge challenge!

## What's a container?
A container is a virtualization of an operating system on top of your existing operating system. You may have heard of a virtual machine; containers are similar. The difference is that containers will share some of the resources available to your working operating system which makes them lightweight and fast. Docker containers are a way of saying "this is all of the software installed in this configuration, and here's a snapshot of it". Logically, this snapshot is called an image. Images can be shared as these snapshots, or they can be "built" by recipe-like files called "Dockerfiles".

In order to improve and share dockerfiles or images, we rely on version control systems and registries. We use version control, like github or bitbucket, for dockerfiles because they are small text-files. This means changes in the overall build can be tracked and committed to history. Registries, like Dockerhub or Microsoft Azure, store compressed images that can be pulled to your computer without having to be built.

## Robots in containers!
Images are *predictable* and *repeatable* development environments. The frustrations with installing ROS and managing dependencies are bound to the Dockerfile which can be maintained by experienced developers. We *don't* need to reinstall to the right ROS version, we *don't* need to reinstall linux, we *don't* need to track down some piece of undocumented software. We can simply grab a container with a robotics project inside and run it on our machine. If it works* on the developers machine it will work* on your machine.

Later, we will see that certain projects were developed and are stable in a given ROS distribution. For example, the Astrobee project requires huge amounts of setup in an older linux distribution to begin experimenting. If you were to install this linux distribution on a new computer, you may have hardware compatibility. If you wanted to use the linux distribution as your main operating system, you may be longing for newer features.

Wayyy later (TODO) we will see that different ROS containers can work together and communicate through standard networks over web. These containers are suitable for swarm or multi-agent robot configurations.

# Installing Docker and Quickstart
Just follow the instructions on the website? input here @TB @SO?
## Windows
1. Please install your OS, then come back here.
2. Download this [file](docker-compose.yml) somewhere you know
3. docker desktop gui to start the docker compose?
4. 
docker desktop install
virtualization
## MacOS
## Linux
If you are using apt for package management, enter the following in your terminal to update your list of available packages. Then the second line to install Docker engine and Containerd for container logistics:
```
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
```
Assuming everything went well, download the following `docker-compose.yml` script for the HOME-STRI robot project and run it.
```
curl https://raw.githubusercontent.com/jdekarske/homestri-ur5e/master/docker-compose.yml > docker-compose.yml && docker-compose up
```

Then open a browser to see inside the container:
```
xdg-open http://localhost:8080/vnc_auto.html
```

# Developing
Now to the fun part, let's play with robots.

## VSCode
Visual Studio Code is a piece of software that makes it quick and easy to write code, debug, and (for us) manage containers.

Download and install [here](https://code.visualstudio.com/).

After opening up the program, you'll see the left bar filled with icons !(initialicons.png). You'll see:
- the explorer, for file browsing
- search, for finding files and text in files
- source control, for working with git
- run, for running and debugging scripts
- extensions, community made vscode addons!

Let's take advantage of some great extensions. Click the icon, search for `Docker` and install, then `Remote - Containers` and install.

> Jason uses these extensions (not all docker related):
> - Bracket Pair Colorizer 2
> - C/C++
> - Docker
> - Doxygen Documentation Generator
> - GitLens
> - Markdown All in One
> - Output Colorizer
> - PlatformIO IDE
> - Python
> - Rainbow CSV
> - Remote*
> - Rewrap
> - ROS

Click the docker whale and you'll see:
- Containers, recently run images
- Images, downloaded images
- Registries, connections to your dockerhub account
- Networks
- Volumes, directories shared with a container

<Assume we're in homestri directory?>

Clone the experiment repository  if you need it and start the container with the browser interface:

```
git clone https://github.com/jdekarske/homestri-ur5e
docker-compose up -d
```

click the green button in the lower left corner of the VSCode window and choose "...Attach to Running Container"

Another window will open where you will be able to work as if you are in the container.

> This tutorial stresses the inclusivity of MacOS, Windows, and Linux for starters, but you will achieve much better performance running docker in Linux. If you are not satisfied with the speed of your containers or if you run into memory issues, consider dual booting.

# Distribution

