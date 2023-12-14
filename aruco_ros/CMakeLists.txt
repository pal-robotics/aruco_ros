cmake_minimum_required(VERSION 3.10.2)
project(aruco_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Public dependencies.
set(dependencies
    OpenCV
    cv_bridge
    geometry_msgs
    image_transport
    rclcpp
    rclpy
    tf2
    tf2_ros
    tf2_geometry_msgs
    aruco
    aruco_msgs
    sensor_msgs
    visualization_msgs
)

find_package(ament_cmake REQUIRED)
foreach(DEP ${dependencies})
  find_package(${DEP} REQUIRED)
endforeach()

# Build library.
add_library(${PROJECT_NAME}_utils SHARED
  src/aruco_ros_utils.cpp
)
target_include_directories(${PROJECT_NAME}_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(${PROJECT_NAME}_utils PUBLIC
  ${OpenCV_LIBRARIES}
  tf2_geometry_msgs::tf2_geometry_msgs
  cv_bridge::cv_bridge
)

target_include_directories(${PROJECT_NAME}_utils PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Build executables.
add_executable(single src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
target_link_libraries(single
  ${PROJECT_NAME}_utils  

  ${aruco_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${visualization_msgs_TARGETS}

  image_transport::image_transport
)
target_include_directories(single PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(double src/simple_double.cpp
                      src/aruco_ros_utils.cpp)
target_link_libraries(double
  ${PROJECT_NAME}_utils

  ${aruco_LIBRARIES}
  ${OpenCV_LIBRARIES}

  image_transport::image_transport
)
target_include_directories(double PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
target_link_libraries(marker_publisher
  ${PROJECT_NAME}_utils
  
  ${aruco_LIBRARIES}
  ${aruco_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
  
  image_transport::image_transport
)
target_include_directories(marker_publisher PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_msgs_INCLUDE_DIRS}
)

# Install library and resources.
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}_utils
  EXPORT export_${PROJECT_NAME}_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS single
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS double
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Exports.
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
