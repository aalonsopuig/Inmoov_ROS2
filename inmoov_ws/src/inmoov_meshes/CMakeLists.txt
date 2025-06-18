cmake_minimum_required(VERSION 3.5)
project(inmoov_meshes)

find_package(ament_cmake REQUIRED)

# Instalar solo el directorio meshes, que contiene los archivos STL/DAE/URDF necesarios
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
