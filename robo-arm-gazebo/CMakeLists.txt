cmake_minimum_required(VERSION 3.5)
project(robo-arm-gazebo)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()
