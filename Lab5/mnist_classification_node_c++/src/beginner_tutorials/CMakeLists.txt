cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
   cv_bridge
)
find_package(OpenCV)

add_message_files(
   FILES
   Num.msg
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


include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB helloworld_SRC CONFIGURE_DEPENDS "*.h" "*.cpp")
add_executable(talker src/talker.cpp include/nn/networks/graph.h include/utils.h src/utils.cpp src/nn/networks/graph.cpp
         src/nn/networks/graph.cpp src/nn/networks/edge.cpp src/nn/networks/vertex.cpp
        include/nn/networks/vertex.h include/nn/weight_initializer.h src/nn/weight_initializer.cpp include/nn/weight_optimizer.h src/nn/weight_optimizer.cpp
        src/nn/networks/graph_utility_prop_implementations.cpp src/nn/architecture_initializer.cpp )
target_link_libraries(talker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)