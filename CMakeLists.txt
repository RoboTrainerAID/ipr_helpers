cmake_minimum_required(VERSION 2.8.3)
project(ipr_helpers)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure geometry_msgs rosparam_handler std_msgs tf2 tf2_geometry_msgs tf2_ros)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" Cpp11CompilerFlag)
if (${Cpp11CompilerFlag})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pose2DStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs  # Or other packages containing msgs
)

generate_ros_parameter_files(
  cfg/ExtractMobileRobotTF.params
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ipr_helpers
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs rosparam_handler std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(extract_mobile_robot_tf src/extract_mobile_robot_tf.cpp)
add_dependencies(extract_mobile_robot_tf ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_genparam)
target_link_libraries(extract_mobile_robot_tf
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS extract_mobile_robot_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
