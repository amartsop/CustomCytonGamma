cmake_minimum_required(VERSION 3.0.2)
project(cyton_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs actionlib moveit_ros_planning_interface
  moveit_msgs control_msgs)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs actionlib 
  moveit_ros_planning_interface moveit_msgs control_msgs)

# # Eigen linking
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#******************** Build ***********************#
# Source files variable
set(PROJ_SOURCE_FILES
    src/euler_rotations.cpp
    src/robot_control.cpp
    src/path_planning.cpp)
    
# Directories variable
set(PROJ_INCL_DIRS include include/cyton_control
  /usr/local/include/eigen3
  ${catkin_INCLUDE_DIRS})

set(PROJ_LIBS ${catkin_LIBRARIES})

# Include directories to the project
include_directories(${PROJ_INCL_DIRS})

# Add executables to target sync_test_node
add_executable(${PROJECT_NAME}_node src/cyton_control_node.cpp
  ${PROJ_SOURCE_FILES})

# Link libraries to target robot_manipulation_node
target_link_libraries(${PROJECT_NAME}_node ${PROJ_LIBS})