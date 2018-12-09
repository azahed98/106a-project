# Install script for directory: /home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/msg" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/srv" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/devel/include/reflex_one_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/devel/share/common-lisp/ros/reflex_one_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/devel/lib/python2.7/dist-packages/reflex_one_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/devel/lib/python2.7/dist-packages/reflex_one_msgs")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgs-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgsConfig.cmake"
    "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/build/reflex-ros-pkg/reflex_one_msgs/catkin_generated/installspace/reflex_one_msgsConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_one_msgs" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-abz/Downloads/project/src/reflex-ros-pkg/reflex_one_msgs/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

