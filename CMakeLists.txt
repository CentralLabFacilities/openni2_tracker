cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS 
					roscpp
					roslib
                                        roslaunch
                                        pluginlib
					image_transport
					tf
                                        actionlib
                                        openni2_tracker_msgs)
# Find OpenNI2
find_package(PkgConfig)
pkg_check_modules(OpenNI2 REQUIRED libopenni2)

roslaunch_add_file_check(launch)

set(NITE2_INCLUDE $ENV{PREFIX}/share//NiTE-2.0.0/Include)
set(NITE2_REDIST $ENV{PREFIX}/share//NiTE-2.0.0/Redist)

# Find Nite2
message(STATUS "NITE2_INCLUDE: " ${NITE2_INCLUDE})
message(STATUS "NITE2_REDIST64: " ${NITE2_REDIST})

find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS ${NITE2_INCLUDE})
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS ${NITE2_REDIST}
	     PATH_SUFFIXES lib) 

catkin_package(
  CATKIN_DEPENDS roscpp openni2_tracker_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDE_DIRS}
  ${Nite2_INCLUDEDIR})

link_directories(${OpenNI2_LIBRARY_DIRS})
add_executable(openni2_tracker src/tracking.cpp src/openni2_tracker.cpp)

target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
