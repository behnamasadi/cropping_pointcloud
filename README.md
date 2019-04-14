# Cropping Pointcloud
This ROS package enables you to crop the scene from Kinect (input topic type: pcl pointcloud) with ROS dynamic reconfigure. You can even enable fitting a plane to remove the ground from the scene and by adjusting correct parameter you can get the desired object from the scene.


![GUI in dynamic reconfigure ](/images/cropping_pointcloud_dynamic_reconfigure_gui.jpg)
![White dots are original scene and rgb dots are from the cropped cloud. Values for the volume of cuboid are coming from sliders.](/images/cropping_pointcloud_rviz.jpg)

[Refrence](http://ros-developer.com/2017/12/04/gui-ros-package-cropping-pcl-pointcloud-dynamic-reconfigure/)
 
![alt text](https://img.shields.io/badge/license-BSD-blue.svg)
[![Build Status](https://travis-ci.org/behnamasadi/cropping_pointcloud.svg?branch=master)](https://travis-ci.org/behnamasadi/cropping_pointcloud)
