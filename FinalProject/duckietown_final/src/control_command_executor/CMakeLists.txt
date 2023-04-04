cmake_minimum_required(VERSION 3.0.2)
project(control_commands)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(OpenCV)

add_message_files(
   FILES
   WheelsCmd.msg
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



add_executable(control_commands src/control_commands.cpp)
target_link_libraries(control_commands ${catkin_LIBRARIES})
add_dependencies(control_commands control_commands_generate_messages_cpp)