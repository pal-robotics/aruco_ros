cmake_minimum_required(VERSION 3.0.0)
project(aruco)

find_package(catkin)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
string(REPLACE "-Werror=shadow" "" NEW_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${NEW_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
)
include_directories(
  include/aruco/)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(aruco
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
target_link_libraries(aruco
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############

install(TARGETS aruco
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

