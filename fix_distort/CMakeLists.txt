cmake_minimum_required(VERSION 3.0.2)
project(fix_distort)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosconsole
  roscpp
  sensor_msgs 
 ## pcl_conversions
 ## pcl_ros
)



catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## add_executable(my_node src/my_node.cpp)
## target_link_libraries(my_node ${catkin_LIBRARIES})


