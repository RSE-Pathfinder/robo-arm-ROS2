cmake_minimum_required(VERSION 3.5)
project(robo-arm-bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
