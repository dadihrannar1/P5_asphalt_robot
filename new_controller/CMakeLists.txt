cmake_minimum_required(VERSION 3.0.2)
project(new_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs message_generation tf moveit_ros_planning_interface)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
  add_message_files(
    FILES
  )

## Generate services in the 'srv' folder
  add_service_files(
    FILES
  )

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES webots_ros
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime tf
#  DEPENDS system_lib
)


set(WEBOTS_HOME /usr/local/webots)
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

set(WEBOTS_INCLUDE
${WEBOTS_HOME}/include/controller/c/
${WEBOTS_HOME}/include/controller/c/utils
${WEBOTS_HOME}/include/controller/cpp/
${WEBOTS_HOME}/include/controller/cpp/utils
)

include_directories( 
${catkin_INCLUDE_DIRS}
${WEBOTS_INCLUDE}
)


## Instructions for keyboard_teleop node

add_executable(manipulator controllers/manipulator_controller.cpp)

add_dependencies(manipulator new_controller_generate_messages_cpp)

target_link_libraries(manipulator
	${catkin_LIBRARIES} ${LIBRARIES}
)




#############
## Install ##
#############

install(TARGETS manipulator
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(TARGETS new_controller
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS scripts/ros_controller.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS scripts/ros_python.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})#

catkin_install_python(PROGRAMS scripts/webots_launcher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
