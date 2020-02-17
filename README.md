# homestri-ur5e
My progress developing an experiment using a UR5-e robot arm for the HOME STRI.

Install docker
```
sudo apt-get install curl
curl -sSL https://get.docker.com/ | sh
sudo usermod -aG docker $(whoami)
```
Clone repo for a helper script
```
git clone https://github.com/jdekarske/homestri-ur5e
```
Pull the container
```
docker pull jdekarske/homestri-ur5e
```
Run the container
```
./homestri-ur5e/gui-docker --rm -it rosdocker:latest
```
