cmake_minimum_required(VERSION 3.5)
project(turtlebot3_mazesolver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
ament_package()
