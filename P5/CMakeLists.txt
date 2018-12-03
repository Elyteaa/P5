cmake_minimum_required(VERSION 2.8.3)
project(P5)

## Find catkin and any catkin packagescc
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

##Enable Python module support
catkin_python_setup()

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES)
#add_action_files(FILES)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
#catkin_install_python(PROGRAMS scripts/positionsMain
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#catkin_install_python(PROGRAMS scripts/robotMain
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##add_executable(tower src/positionsMain.py)
##target_link_libraries(tower ${catkin_LIBRARIES})
##add_dependencies(tower sensing_system_generate_messages_cpp)

##add_executable(robot src/robotMain.py)
##target_link_libraries(robot ${catkin_LIBRARIES})
##add_dependencies(robot sensing_system_generate_messages_cpp)