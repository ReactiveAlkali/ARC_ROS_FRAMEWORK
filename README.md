# arc\_ros
Framework for distributed coordination and recruitment mechanisms for robotic teams on the ROS platform.

## Installation

Currently this project is built for ROS Kinetic and does not work with newer ROS versions.  ROS 
Kinetic requires it to be installed on Ubuntu 16.04 which I recommend running in a virtual machine.
After you have finished installing Ubuntu 16.04 and ROS Kinetic you will additionally need to install
the following ROS packages:

* fake\_localization
* map\_server
* marker\_msgs
* move\_base

### Environment Setup

Run the following commands to setup your arc\_ros environment. For additional comments on these
commands please refer to the documentation.


```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/workspace/arc_ws/src
cd ~/workspace/arc_ws/src
catkin_init_workspace
cd ../
source devel/setup.bash
cd src
git clone https://github.com/ReactiveAlkali/ARC_ROS_FRAMEWORK.git
```

### Building

To build the project you will need to run `catkin_make` from within your arc\_ws directory. If
it fails to compile it likely means you need to compile the arc\_msgs package first. This can be 
done by running `catkin_make --pkg arc_msgs`
