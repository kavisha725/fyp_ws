# fyp_ws
ROS packages used in our final year project robot. Fully tested on an SBC with Ubuntu 16.04LTS and ROS Kinetic.

---

## Project overview

The two-wheeled self-balancing Personal Mobility Vehicle capable of manual, tele-op and autonomous navigation. 

![](https://raw.githubusercontent.com/kavisha725/fyp_ws/master/pics/results.png) 

---

## Robot Platform

CAD design. 

![](https://raw.githubusercontent.com/kavisha725/fyp_ws/master/pics/platform.jpg) 

---

## System architecture

Block diagram.

![](https://raw.githubusercontent.com/kavisha725/fyp_ws/master/pics/block_diagram.png) 

---

## Navigation stack

Modifications to the typical ROS navigation stack. 

![](https://raw.githubusercontent.com/kavisha725/fyp_ws/master/pics/ROS_custom_nav_stack.png) 

---

## Robot startup scripts

  To launch base_controller, rosserial and odometry filter:  
  ```
  roslaunch rob_loc robot_localization.launch
  ```

  To launch interface with the android app:  
  ```
  roslaunch app_interface app_interface.launch
  ```
  
   To launch the lidar, Kinect, [amcl](http://wiki.ros.org/amcl) and [move_base](http://wiki.ros.org/move_base) with pre-built map:  
  ```
  roslaunch nav_stack move_base.launch
  ```

---

