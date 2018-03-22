cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_can_msg_filters)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  can_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp can_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
