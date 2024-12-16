cmake_minimum_required(VERSION 3.0.2)
project(ur_python)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  message_runtime
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package (OpenCV REQUIRED)

add_message_files(
  FILES
  object_info.msg
  robot_state.msg
  grip_command.msg
  grip_state.msg
  pet_info.msg
  new_stone.msg
  capture_flag.msg
  capture_finished_flag.msg
  Location.msg
  Coordinate.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES opencv
    CATKIN_DEPENDS 
    cv_bridge 
		image_transport
    sensor_msgs 
		std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# For omoks.cpp file
add_executable(go src/omoks.cpp)
target_link_libraries(go ${catkin_LIBRARIES})

#############
## Install ##
#############
