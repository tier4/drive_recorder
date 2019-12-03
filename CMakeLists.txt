cmake_minimum_required(VERSION 2.8.3)
project(drive_recorder)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ros_observer
)

catkin_package(
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#
#buld
#
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(drive_recorder_node src/drive_recorder_node.cpp)

add_executable(drive_recorder_test src/drive_recorder_test.cpp)

target_link_libraries(
  drive_recorder_node
  ${catkin_LIBRARIES}
  rt ${Boost_LIBRARIES}
)

#
#test
#

target_link_libraries(
  drive_recorder_test
  ${catkin_LIBRARIES}
  rt ${Boost_LIBRARIES}
)



