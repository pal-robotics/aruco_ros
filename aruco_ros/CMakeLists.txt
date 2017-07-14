cmake_minimum_required(VERSION 2.8.3)
project(aruco_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  tf
  aruco
  aruco_msgs
)

find_package(OpenCV 2.4 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros
  CATKIN_DEPENDS roscpp aruco
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(aruco_ros
  src/aruco_ros_utils.cpp)

add_executable(single src/simple_single.cpp)
add_dependencies(single aruco_ros ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(single aruco_ros ${catkin_LIBRARIES})

add_executable(double src/simple_double.cpp)
add_dependencies(double aruco_ros ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(double aruco_ros ${catkin_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp)
add_dependencies(marker_publisher aruco_ros ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher aruco_ros ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS single double marker_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

