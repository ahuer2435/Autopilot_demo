cmake_minimum_required(VERSION 2.8.3)
project(dataspeed_can_msgs)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(INCLUDE_DIRS include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY bmr
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
