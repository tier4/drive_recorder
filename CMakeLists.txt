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

#
#buld
#
include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(drive_recorder_node src/drive_recorder_node.cpp)

add_executable(test_drive_recorder test/test_drive_recorder.cpp)

target_link_libraries(
  drive_recorder_node
  ${catkin_LIBRARIES}
  rt ${Boost_LIBRARIES}
)

#
# Install
#

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

## Install executables and/or libraries
install(TARGETS drive_recorder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install project namespaced headers
install(DIRECTORY include/drive_recorder/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#
#test
#

target_link_libraries(
  test_drive_recorder
  ${catkin_LIBRARIES}
  rt ${Boost_LIBRARIES}
)



