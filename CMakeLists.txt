cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  pal_vision_segmentation
  roscpp
  tf
)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_transport pal_vision_segmentation roscpp tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(aruco
              src/aruco/arucofidmarkers.cpp
              src/aruco/cvdrawingutils.cpp
              src/aruco/cameraparameters.cpp
              src/aruco/board.cpp
              src/aruco/marker.cpp
              src/aruco/boarddetector.cpp
              src/aruco/markerdetector.cpp
              src/aruco/ar_omp.cpp
              src/aruco/subpixelcorner.cpp
)
target_link_libraries(aruco ${catkin_LIBRARIES})

add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(single aruco ${catkin_LIBRARIES})

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
add_dependencies(double ${PROJECT_NAME}_gencfg)
target_link_libraries(double aruco ${catkin_LIBRARIES})


install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
