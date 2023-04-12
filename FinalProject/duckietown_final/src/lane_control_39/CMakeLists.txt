cmake_minimum_required(VERSION 3.0.2)
project(lane_control_39)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  image_transport
  cv_bridge
)
find_package(OpenCV)

add_message_files(
   FILES
   WheelsCmd.msg
 )

 add_service_files(
   FILES
    AddTwoInts.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )
## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)
include_directories("/usr/local/include/apriltag/")


include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(lane_control src/lane_control.cpp)
target_link_libraries(lane_control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lane_control "/usr/local/lib/libapriltag.so")
add_dependencies(lane_control lane_control_generate_messages_cpp)