cmake_minimum_required(VERSION 2.8.3)
project(track)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES track
#  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(track_turtlebot src/track_turtlebot.cpp src/slave_functions.cpp)
 target_link_libraries(track_turtlebot ${catkin_LIBRARIES})

 add_executable(master src/master.cpp)
 target_link_libraries(master ${catkin_LIBRARIES})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
 install(TARGETS master track_turtlebot
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )
