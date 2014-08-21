cmake_minimum_required(VERSION 2.8.3)
project(battery_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diagnostic_msgs
  diagnostic_updater
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs diagnostic_updater
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(battery_manager src/battery_manager.cpp)
add_dependencies(battery_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(battery_manager
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS battery_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY demo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
