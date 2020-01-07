cmake_minimum_required(VERSION 2.8.3)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

project(catch_control)
#add_compile_options(-std=c++11)
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
  trac_ik_lib
  trajectory_msgs
  rosbag
        message_generation
        robotiq_85_msgs
        eigen_conversions
        eigen_stl_containers
        eigen_typekit
        geometry_msgs
        moveit_ros_planning_interface
        moveit_visual_tools
        serial

)

#add_message_files(
#        FILES
#        MyoArm.msg
#        MyoPose.msg
#        EmgArray.msg
#)

generate_messages(
        DEPENDENCIES
        std_msgs
)



catkin_package(

  CATKIN_DEPENDS
        robotiq_85_msgs
        moveit_visual_tools

)



include_directories(include
  ${catkin_INCLUDE_DIRS}
)


#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIR})

add_executable(test1 src/catch_bottle.cpp src/catch_bottle.h simulink2C/end_point_filter/end_point_filter.cpp simulink2C/force_interaction/force_interaction.cpp)
target_link_libraries(test1 ${catkin_LIBRARIES})


add_executable(joint_state_maintain_node src/joint_state_maintain.cpp src/joint_state_maintain.h)
add_dependencies(joint_state_maintain_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joint_state_maintain_node ${catkin_LIBRARIES})

add_executable(join_two_trajectory_node src/join_two_trajectory.cpp src/join_two_trajectory.h)
add_dependencies(join_two_trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(join_two_trajectory_node ${catkin_LIBRARIES})


add_executable(record_data_matlab_node src/record_data_matlab.cpp)
add_dependencies(record_data_matlab_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_data_matlab_node ${catkin_LIBRARIES})

add_executable(match_demo_node src/match_demo.cpp src/match_demo.h)
add_dependencies(match_demo_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(match_demo_node ${catkin_LIBRARIES})


add_executable(Serial src/Serial.cpp)
add_dependencies(Serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Serial ${catkin_LIBRARIES})

add_executable(listener_collect_force_node src/listener.cpp)
add_dependencies(listener_collect_force_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(listener_collect_force_node ${catkin_LIBRARIES})


add_executable(serialport_cmd src/serialport_cmd.cpp)
add_dependencies(serialport_cmd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serialport_cmd ${catkin_LIBRARIES})
