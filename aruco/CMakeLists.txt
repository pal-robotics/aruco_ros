cmake_minimum_required(VERSION 3.8)
project(aruco)

set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
string(REPLACE "-Werror=shadow" "" NEW_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${NEW_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/aruco/cvdrawingutils.cpp
  src/aruco/cameraparameters.cpp
  src/aruco/cvdrawingutils.cpp
  src/aruco/debug.cpp
  src/aruco/dictionary_based.cpp
  src/aruco/dictionary.cpp
  src/aruco/fractaldetector.cpp
  src/aruco/ippe.cpp
  src/aruco/marker.cpp
  src/aruco/markerdetector_impl.cpp
  src/aruco/markerdetector.cpp
  src/aruco/markerlabeler.cpp
  src/aruco/markermap.cpp
  src/aruco/posetracker.cpp
  src/aruco/fractallabelers/fractallabeler.cpp
  src/aruco/fractallabelers/fractalmarker.cpp
  src/aruco/fractallabelers/fractalmarkerset.cpp
  src/aruco/fractallabelers/fractalposetracker.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/aruco>
  $<INSTALL_INTERFACE:include/aruco>
)

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
   DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
