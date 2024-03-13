cmake_minimum_required(VERSION 3.5)
project(turtlebot3_mazesolver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


install(
  DIRECTORY launch config urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/teleop_gamepad.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
