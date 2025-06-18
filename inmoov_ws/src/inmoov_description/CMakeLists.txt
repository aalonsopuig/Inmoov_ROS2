cmake_minimum_required(VERSION 3.5)
project(inmoov_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
