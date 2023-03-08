execute_process(COMMAND "/home/khurramjaved/dt-ros-commons/build/ros_http_api/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/khurramjaved/dt-ros-commons/build/ros_http_api/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
