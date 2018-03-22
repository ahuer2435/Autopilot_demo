cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_can_tools)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  roslib
  std_msgs
  can_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/DbcIterator.cpp src/Message.cpp src/Signal.cpp
  src/CanExtractor.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(dbc_node src/dbc_node.cpp)
target_link_libraries(dbc_node
  ${PROJECT_NAME}
)

add_executable(dbc_bag src/dbc_bag.cpp)
target_link_libraries(dbc_bag
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} dbc_node dbc_bag
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
