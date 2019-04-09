cmake_minimum_required(VERSION 2.8.3)
project(nav_scripts)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    genmsg
    actionlib_msgs
    std_msgs
    geometry_msgs
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   Basetasks.srv
 )

# Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES Navigator.action
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   actionlib_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_scripts
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
)

