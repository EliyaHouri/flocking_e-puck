# Install script for directory: /home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/argos3/plugins/robots/e-puck2/control_interface" TYPE FILE FILES
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_proximity_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_light_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_ground_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/argos3/plugins/robots/e-puck2/simulator" TYPE FILE FILES
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/dynamics2d_epuck2_model.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_led_equipped_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_tof_equipped_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_encoder_equipped_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_proximity_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_light_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_ground_rotzonly_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_led_default_actuator.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_tof_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_colored_blob_perspective_camera_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_battery_equipped_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_camera_equipped_entity.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_battery_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/epuck2_encoder_default_sensor.h"
    "/home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2/simulator/qtopengl_epuck2.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/argos3" TYPE SHARED_LIBRARY FILES "/home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2/libargos3plugin_simulator_epuck2.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so"
         OLD_RPATH "/usr/local/lib/argos3:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/argos3/libargos3plugin_simulator_epuck2.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

