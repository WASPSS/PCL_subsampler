cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_point_cloud_subsampler)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  rospy
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_subsampler src/point_cloud_subsampler.cpp)
target_link_libraries(point_cloud_subsampler ${catkin_LIBRARIES})

add_executable(PCL_transform src/PCL_transform.cpp)
target_link_libraries(PCL_transform ${catkin_LIBRARIES})
