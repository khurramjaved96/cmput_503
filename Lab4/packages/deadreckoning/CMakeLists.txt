cmake_minimum_required(VERSION 2.8.3)
project(deadreckoning)

find_package(catkin REQUIRED COMPONENTS
 roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  tf_conversions
  tf
  # duckietown_msgs # Every duckietown packages should use this.
)



catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

