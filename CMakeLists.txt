cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS 
					roscpp
					roslib
					nodelet
                                        pluginlib
					image_transport
					tf
                                        actionlib
                                        openni2_tracker_msgs)
# Find OpenNI2
find_package(PkgConfig)
pkg_check_modules(OpenNI2 REQUIRED libopenni2)

add_action_files(
	DIRECTORY action 
	FILES NiteTracker.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   openni2_tracker_msgs
)

# Find Nite2
message(STATUS "NITE2_INCLUDE: " $ENV{NITE2_INCLUDE})
message(STATUS "NITE2_REDIST64: " $ENV{NITE2_REDIST})

set(NITE2_INCLUDE ${CMAKE_INSTALL_PREFIX}/share//NiTE-2.0.0/Include)
set(NITE2_REDIST ${CMAKE_INSTALL_PREFIX}/share//NiTE-2.0.0/Redist)

find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS ${NITE2_INCLUDE})
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS ${NITE2_REDIST}
	     PATH_SUFFIXES lib) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDE_DIRS}
  ${Nite2_INCLUDEDIR})
link_directories(${OpenNI2_LIBRARY_DIRS})
add_library(openni2_tracker src/openni2_tracker_nodelet.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})
add_dependencies(openni2_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES openni2_tracker_nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/rosjava_messages/ 
DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/ros/rosjava_messages)
