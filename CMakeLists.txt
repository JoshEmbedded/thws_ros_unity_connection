cmake_minimum_required(VERSION 3.0.2)
project(weldingrobot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  moveit_ros_planning_interface
  moveit_core
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES weldingrobot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# find_package(catkin REQUIRED COMPONENTS
#   ros_tcp_endpoint
# )

# catkin_package(
#   CATKIN_DEPENDS ros_tcp_endpoint
# )

## Declare weldingPath.cpp as a library
add_library(weldingPath_lib src/weldingPath.cpp)

## add_library(moveit_error_handler src/moveit_error_handler.cpp)

## Link the library to the executable
target_link_libraries(weldingPath_lib
    ${catkin_LIBRARIES}
    ${moveit_core_LIBRARIES}
    ${moveit_ros_planning_interface_LIBRARIES}
)

## Declare a C++ executable
add_executable(pose_send src/pose_send.cpp)
add_executable(unity_listener src/unity_listener.cpp)
add_executable(unity_sphere_follow src/unity_sphere_follow.cpp)

## Add cmake target dependencies of the executable
add_dependencies(pose_send ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} weldingPath_lib)
add_dependencies(unity_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(unity_sphere_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## add_dependencies(moveit_error_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link the library to the executable
target_link_libraries(pose_send weldingPath_lib ${catkin_LIBRARIES})
target_link_libraries(unity_listener ${catkin_LIBRARIES})
target_link_libraries(unity_sphere_follow ${catkin_LIBRARIES})


## target_link_libraries(moveit_error_handler ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/csvPoints.py  # Update with the actual path to your Python script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
