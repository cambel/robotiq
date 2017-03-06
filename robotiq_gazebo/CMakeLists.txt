cmake_minimum_required(VERSION 2.8.3)
project(robotiq_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  control_toolbox
  xmlrpcpp
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    control_toolbox
    xmlrpcpp
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(gazebo_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
