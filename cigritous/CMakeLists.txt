cmake_minimum_required(VERSION 3.5)

project(cigritous)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(vio_bridge 
  src/vio_bridge.cpp)
ament_target_dependencies(vio_bridge
  rclcpp
  nav_msgs
  tf2
  px4_msgs)

ament_export_dependencies(ament_cmake ament_cmake_python rclcpp rclpy)

# Install Cpp executables
install(TARGETS
  vio_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(module)

install(PROGRAMS
  scripts/vision.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
