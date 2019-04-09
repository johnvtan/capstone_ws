cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_republish)
                                                                                                                        
## Compile as C++11, supported in ROS Kinetic and newer                                                                 
add_compile_options(-std=c++11)                                                                                         
                                                                                                                        
find_package(catkin REQUIRED COMPONENTS                                                                                 
  pcl_ros                                                                                                               
  roscpp                                                                                                                
)                                                                                                                       
                                                                                                                        
find_package(PCL REQUIRED)                                                                                              
                                                                                                                        
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
                                                                                                                        
add_executable(pointcloud_republish_node src/pointcloud_republish_node.cpp)                                                   
target_link_libraries(pointcloud_republish_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})  
