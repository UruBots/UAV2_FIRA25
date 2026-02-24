# Install script for directory: /home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jetson/UAV2/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces/msg" TYPE FILE FILES
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/BoundingBox2Df.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/BoundingBox2Di.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/BoundingBox3D.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Keypoint2Df.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Keypoint2Di.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Keypoint3D.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Object.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/ObjectsStamped.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Skeleton2D.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/Skeleton3D.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/RGBDSensors.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/PlaneStamped.msg"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/msg/PosTrackStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces/srv" TYPE FILE FILES
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/set_pose.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/reset_odometry.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/reset_tracking.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/start_svo_recording.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/stop_svo_recording.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/start_remote_stream.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/stop_remote_stream.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/set_led_status.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/toggle_led.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/start_3d_mapping.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/stop_3d_mapping.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/save_3d_map.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/start_object_detection.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/stop_object_detection.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/save_area_memory.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/set_roi.srv"
    "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/srv/reset_roi.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces/cmake" TYPE FILE FILES "/home/jetson/UAV2/build/zed-ros-wrapper/zed-ros-interfaces/catkin_generated/installspace/zed_interfaces-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jetson/UAV2/devel/include/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jetson/UAV2/devel/share/roseus/ros/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jetson/UAV2/devel/share/common-lisp/ros/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jetson/UAV2/devel/share/gennodejs/ros/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/jetson/UAV2/devel/lib/python3/dist-packages/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jetson/UAV2/devel/lib/python3/dist-packages/zed_interfaces")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jetson/UAV2/build/zed-ros-wrapper/zed-ros-interfaces/catkin_generated/installspace/zed_interfaces.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces/cmake" TYPE FILE FILES "/home/jetson/UAV2/build/zed-ros-wrapper/zed-ros-interfaces/catkin_generated/installspace/zed_interfaces-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces/cmake" TYPE FILE FILES
    "/home/jetson/UAV2/build/zed-ros-wrapper/zed-ros-interfaces/catkin_generated/installspace/zed_interfacesConfig.cmake"
    "/home/jetson/UAV2/build/zed-ros-wrapper/zed-ros-interfaces/catkin_generated/installspace/zed_interfacesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces" TYPE FILE FILES "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/zed_interfaces" TYPE DIRECTORY FILES "/home/jetson/UAV2/src/zed-ros-wrapper/zed-ros-interfaces/meshes")
endif()

