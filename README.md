# TRAIL

TRAIL is a DRL-VO navigation approach created at Temple University with the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html) in mind.  
This repository contains the TRAIL policy modified to function within arena-rosnav platform.  
Original repository can be found [here](https://github.com/TempleRAIL/nav-competition-icra2022-drl-vo)

# Installation
## Add this package to your .rosinstall or clone it manually.
```bash
cd ~/catkin_ws/src/arena-bench # Navigate to your arena-bench location
echo "- git:
    local-name: ../planners/trail
    uri: https://github.com/Arena-Rosnav/trail
    version: master" >> .rosinstall
rosws update ../planners/trail # or rosws update
```
## Activate poetry shell
```bash
cd ~/catkin_ws/src/arena-bench # Navigate to your arena-bench location
poetry shell
```
## Make sure to source the workspace environment
```bash
cd ../.. # navigate to the catkin_ws directory
catkin_make
source devel/setup.zsh # if you use bash: source devel/setup.bash 
```
