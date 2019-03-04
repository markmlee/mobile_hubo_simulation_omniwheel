cmake_minimum_required(VERSION 2.8.3)
project(hubo_plugin)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_msgs #mark removed
  std_msgs
  message_generation
  tf
)


generate_messages(DEPENDENCIES std_msgs)


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

add_library(drc_plugin src/drc_plugin.cpp)
target_link_libraries(drc_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(hubo_gazebo_plugin src/hubo_gazebo_plugin.cpp)
target_link_libraries(hubo_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    std_msgs
)
