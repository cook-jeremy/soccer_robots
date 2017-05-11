# Soccer Robots
This repositort contains all top-level ROS packages for my autonomous soccer robots project. The size and shape of these robots are cube shaped with dimensions approximately 11.5x11.5x9 cm^3. All the robot parts are 3D printed, with the only exceptions for the wheels and tracks which were purchased from Pololu. The robots positions are tracked using OpenCV color recognition, and commands are sent to the robots via UDP packets.

<p align="center">
  <img src="http://i.imgur.com/n0Dbfh6.jpg" width="400"/>
  <img src="http://i.imgur.com/T1JxGsI.jpg" width="400"/>
  <img src="http://i.imgur.com/2LMFLc8.jpg" width="400"/>
  <img src="http://i.imgur.com/ACQWQDa.jpg" width="400"/>
</p>

## Installation

### From Source

This project is built for ROS Kinetic on Ubuntu 16.04, which you can [install here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Then, make sure the ROS_DISTRO environment variable is set correctly:

```
echo $ROS_DISTRO
```

It may already be.  If not, issue this shell command:

```
$ export ROS_DISTRO=kinetic
```

Next, create a workspace and clone the source repositories:
```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/MontyPylon/soccer_robots.git
```

Install all dependencies:
```
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Then, build everything:
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
If everthing was installed correctly, the project should build with no errors, and is ready to launch!
