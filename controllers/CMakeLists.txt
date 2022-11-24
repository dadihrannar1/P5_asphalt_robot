cmake_minimum_required(VERSION 3.0.2)
project(controllers)






 add_compile_options(-std=c++11)


 #file(GLOB C_SOURCES *.c)
 #file(GLOB CPP_SOURCES *.cpp)
 #set(SOURCES ${C_SOURCES} ${CPP_SOURCES})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  moveit_core
  moveit_ros_planning_interface
  tf
)


add_message_files(

FILES	
vision_out.msg
#   Message1.msg
#   Message2.msg
 )
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


 generate_messages(
  DEPENDENCIES
  std_msgs
  
 )



## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
  #INCLUDE_DIRS include
 #LIBRARIES controllers
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  moveit_core moveit_ros_planning_interface tf
#  DEPENDS system_lib
)




set(WEBOTS_HOME /usr/local/webots)
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

set(WEBOTS_INCLUDE
${WEBOTS_HOME}/include/controller/c
${WEBOTS_HOME}/include/controller/c/utils
${WEBOTS_HOME}/include/controller/cpp
${WEBOTS_HOME}/include/controller/cpp/utils
)

include_directories( 
${catkin_INCLUDE_DIRS}
${WEBOTS_INCLUDE}
/home/dadi/p5_ws/src/P5_asphalt_robot/controllers/Itsa_me_pain_again
/home/dadi/p5_ws/src/P5_asphalt_robot/UDP/include
)

add_executable(sub src/subscriber_test.cpp)

add_dependencies(sub controllers_generate_messages_cpp)

target_link_libraries(sub ${catkin_LIBRARIES})

add_executable(pain Itsa_me_pain_again/Itsa_me_pain_again.cpp)

add_dependencies(pain controllers_generate_messages_cpp)




target_link_libraries(pain ${catkin_LIBRARIES} ${LIBRARIES}  ) #/home/dadi/p5_ws/src/P5_asphalt_robot/controllers/Itsa_me_pain_again/include)






install(TARGETS sub
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS pain
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY controllers
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})