# Install script for directory: /home/khurramjaved/dt-ros-commons/packages/duckietown_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/khurramjaved/dt-ros-commons/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/msg" TYPE FILE FILES
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/AntiInstagramThresholds.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/AprilTagDetection.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/AprilTagDetectionArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/AprilTagsWithInfos.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/BoolStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ButtonEvent.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/CarControl.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/CoordinationClearance.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/CoordinationSignal.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsCodeProfiling.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsCodeProfilingArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosLink.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosLinkArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosNode.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosParameterArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosTopic.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DiagnosticsRosTopicArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DisplayFragment.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DroneControl.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DroneMode.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/NodeParameter.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/DuckiebotLED.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/EncoderStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/EpisodeStart.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/FSMState.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/IntersectionPose.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/IntersectionPoseImg.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/IntersectionPoseImgDebug.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/KinematicsParameters.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/KinematicsWeights.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LanePose.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LEDDetection.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LEDDetectionArray.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LEDDetectionDebugInfo.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LEDInterpreter.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LEDPattern.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LightSensor.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/LineFollowerStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/MaintenanceState.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ObstacleImageDetection.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ObstacleImageDetectionList.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ObstacleType.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ParamTuner.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Pixel.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Pose2DStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Rect.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Rects.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/SceneSegments.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Segment.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/SegmentList.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/SignalsDetection.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/SignalsDetectionETHZ17.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/SourceTargetNodes.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/StopLineReading.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/TagInfo.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/ThetaDotSample.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Trajectory.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/TurnIDandType.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Twist2DStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Vector2D.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/VehicleCorners.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/VehiclePose.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/Vsample.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/WheelEncoderStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/WheelsCmd.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/WheelsCmdStamped.msg"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/msg/WheelsCmdDBV2Stamped.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/srv" TYPE FILE FILES
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/ChangePattern.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/GetVariable.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/IMUstatus.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/LFstatus.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/NodeGetParamsList.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/NodeRequestParamsUpdate.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/SensorsStatus.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/SetCustomLEDPattern.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/SetFSMState.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/SetValue.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/SetVariable.srv"
    "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/srv/ToFstatus.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES "/home/khurramjaved/dt-ros-commons/build/duckietown_msgs/catkin_generated/installspace/duckietown_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/khurramjaved/dt-ros-commons/devel/include/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/khurramjaved/dt-ros-commons/devel/share/roseus/ros/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/khurramjaved/dt-ros-commons/devel/share/common-lisp/ros/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/khurramjaved/dt-ros-commons/devel/share/gennodejs/ros/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/khurramjaved/dt-ros-commons/devel/lib/python3/dist-packages/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/khurramjaved/dt-ros-commons/devel/lib/python3/dist-packages/duckietown_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/khurramjaved/dt-ros-commons/build/duckietown_msgs/catkin_generated/installspace/duckietown_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES "/home/khurramjaved/dt-ros-commons/build/duckietown_msgs/catkin_generated/installspace/duckietown_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES
    "/home/khurramjaved/dt-ros-commons/build/duckietown_msgs/catkin_generated/installspace/duckietown_msgsConfig.cmake"
    "/home/khurramjaved/dt-ros-commons/build/duckietown_msgs/catkin_generated/installspace/duckietown_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs" TYPE FILE FILES "/home/khurramjaved/dt-ros-commons/packages/duckietown_msgs/package.xml")
endif()

