# homestri-ur5e
My progress developing an experiment using a UR5-e robot arm for the HOME STRI.

Install docker
```
sudo apt-get install curl
curl -sSL https://get.docker.com/ | sh
sudo usermod -aG docker $(whoami)
```
Clone the repo for the gui helper script `gui-docker` (and the linked volumes if you are using them)
```
git clone https://github.com/jdekarske/homestri-ur5e
```
Run this script from the cloned the repository (ex. /home/jason/Documents/homestri-ur5e) and append arguments like you would for any ordinary `docker run` command. For example, I'm linking the volume `experimentdevel` to the container. If this is your first time running this, it may take a few minutes to download the current image.
```
./gui-docker --rm -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-ur5e:rosplan
```
# still testing:
Alternatively, to access the container via browser, use docker-compose with an additional novnc image:
```
docker-compose up -d
```
and access via http://localhost:8080/vnc_auto.html in a web browser.

* works with server on my ubuntu machine, client on windows over LAN (plz test windows<>windows)
* the vnc is debian, not sure if that matters