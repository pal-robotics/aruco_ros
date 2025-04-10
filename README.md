# aruco_ros

Software package and ROS wrappers of the [Aruco](http://www.sciencedirect.com/science/article/pii/S0031320314000235) Augmented Reality marker detector library.

## Features

<img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/marker_in_hand.jpg" /> 

- High-framerate tracking of AR markers
- Generate AR markers with given size and optimized for minimal perceptive ambiguity (when there are more markers to track)
- Enhanced precision tracking by using boards of markers
- ROS wrappers

## Applications

- Object pose estimation
- Visual servoing: track object and hand at the same time
  <img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/reem_gazebo_floating_marker_world.png"/>

## Installation

### Prerequisites

- A compatible Ubuntu version for your chosen ROS distribution
- ROS 2 installed (ROS 1 is approaching EOL)
- OpenCV 4.x (comes with ROS 2 installations)

### Using Binaries

If you prefer to install pre-built binaries, you can use:

```bash
sudo apt install ros-<distro>-aruco-ros
```

Replace `<distro>` with your ROS 2 distribution name (e.g., humble, iron).

### Build from Source

#### 1. Installing ROS

Follow the official ROS 2 installation instructions:
- https://docs.ros.org/

#### 2. Creating a ROS Workspace

```bash
# Create a workspace folder
mkdir -p ~/aruco_ws/src
cd ~/aruco_ws/src
```

#### 3. Clone the aruco_ros Repository

```bash
# Clone the repository
git clone https://github.com/pal-robotics/aruco_ros.git -b <distro>-devel
```

Replace `<distro>` with your ROS 2 distribution name (e.g., humble, iron).

#### 4. Install Dependencies

```bash
# Navigate to the workspace root
cd ~/aruco_ws
# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

#### 5. Build the Workspace

```bash
# Build the workspace using colcon
cd ~/aruco_ws
colcon build
# Source the setup file to update environment
source install/setup.bash
```

### ROS API

#### Messages

- aruco_ros/Marker.msg

       Header header
       uint32 id
       geometry_msgs/PoseWithCovariance pose
       float64 confidence

- aruco_ros/MarkerArray.msg

       Header header
       aruco_ros/Marker[] markers

### Kinetic changes

- Updated the [Aruco](http://www.sciencedirect.com/science/article/pii/S0031320314000235) library to version 3.0.4

- Changed the coordinate system to match the library's, the convention is shown
  in the image below, following rviz conventions, X is red, Y is green and Z is
  blue.
  <img align="bottom" src="/aruco_ros/etc/new_coordinates.png"/>

### Test it with REEM

- Open a REEM in simulation with a marker floating in front of the robot. This will start the stereo cameras of the robot too. Since this is only a vision test, there is nothing else in this world apart from the robot and a marker floating in front of it. An extra light source had to be added to compensate for the default darkness.

  ```bash
  # Launch REEM gazebo simulation with the floating marker world
  roslaunch reem_gazebo reem_gazebo.launch world:=floating_marker
  ```

- Launch the `image_proc` node to get undistorted images from the cameras of the robot.

  ```bash
  # Process images from the right stereo camera to get undistorted images
  ROS_NAMESPACE=/stereo/right rosrun image_proc image_proc image_raw:=image
  ```

- Start the `single` node which will start tracking the specified marker and will publish its pose in the camera frame

  ```bash
  # Launch aruco marker detection with the specified marker ID and size (in meters)
  # 'eye' parameter specifies which camera to use (left or right)
  roslaunch aruco_ros single.launch markerId:=26 markerSize:=0.08 eye:="right"
  ```

  the frame in which the pose is refered to can be chosen with the 'ref_frame' argument. The next example forces the marker pose to
  be published with respect to the robot base_link frame:

  ```bash
  # Launch with a reference frame specified - useful for transforming marker
  # coordinates to a different frame like the robot's base_link
  roslaunch aruco_ros single.launch markerId:=26 markerSize:=0.08 eye:="right" ref_frame:=/base_link
  ```

- Visualize the result

  ```bash
  # View the detection results with image_view to see marker overlay
  rosrun image_view image_view image:=/aruco_single/result
  ```

<img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/reem_gazebo_floating_marker.png"/>