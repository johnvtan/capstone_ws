cmake_minimum_required(VERSION 2.8.3)
project(landing_site_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  dynamic_reconfigure
)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options (
    cfg/LSD.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_test
#  CATKIN_DEPENDS pcl_ros roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(landing_site_detection src/landing_site_detection.cpp)
add_dependencies(landing_site_detection ${PROJECT_NAME}_gencfg)
target_link_libraries(landing_site_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
