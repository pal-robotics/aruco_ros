aruco_ros
=========

Software package and ROS wrappers of the [ArUco][1] Augmented Reality marker detector library.


### Features
<img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/marker_in_hand.jpg" />

 * High-framerate tracking of AR markers
 
 * Generate AR markers with given size and optimized for minimal perceptive ambiguity (when there are more markers to track)
 
 * Enhanced precision tracking by using boards of markers
 
 * ROS wrappers


### Applications

 * Object pose estimation
 * Robot Localization
 * Visual servoing: track object and hand at the same time

<img align="center" src="/aruco_ros/etc/tiago_aruco.png"/>


### ROS API

#### Messages

 * aruco_msg/Marker.msg

        Header header
        uint32 id
        geometry_msgs/PoseWithCovariance pose
        float64 confidence

 * aruco_msg/MarkerArray.msg

        Header header
        aruco_ros/Marker[] markers

### Changelog

* Updated the [ArUco][1] library to version 3.1.5

* Add support for extrinsics with stereo cameras 

* Undistort fisheye detected points

### ArUco marker coordinate system

<img align="bottom" src="/aruco_ros/etc/new_coordinates.png"/>

### Results with TIAGo

<img src="https://media3.giphy.com/media/gRg0FfVUK9viRXp43H/giphy.gif"/>

<img src="https://media2.giphy.com/media/K2ZQXoG8Q5RRgd616h/giphy.gif"/>

<img src="https://media0.giphy.com/media/5hM9fugNr2aFOHHxsI/giphy.gif"/>


[1]: http://www.sciencedirect.com/science/article/pii/S0031320314000235 "Automatic generation and detection of highly reliable fiducial markers under occlusion by S. Garrido-Jurado and R. Muñoz-Salinas and F.J. Madrid-Cuevas and M.J. Marín-Jiménez 2014"
