cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

set(THIS_PACKAGE_INCLUDE_DEPENDS
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils SHARED src/aruco_ros_utils.cpp)
target_include_directories(aruco_ros_utils
  PUBLIC
  include)

target_include_directories( aruco_ros_utils
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_ros_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(aruco_ros_utils ${OpenCV_LIBRARIES})

# add_executable(single src/simple_single.cpp
#                       src/aruco_ros_utils.cpp)
# ament_target_dependencies(single ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(single ${OpenCV_LIBRARIES})

# add_executable(double src/simple_double.cpp
#                       src/aruco_ros_utils.cpp)
# ament_target_dependencies(double ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(double ${OpenCV_LIBRARIES})

# add_executable(marker_publisher src/marker_publish.cpp
#                                 src/aruco_ros_utils.cpp)
# ament_target_dependencies(marker_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco_ros_utils #single double marker_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()


