# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(speak_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(pybind11 REQUIRED)

# Include directories
include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}
)

# Create the executable
add_executable(speak_node
  src/speak_node.cpp
)
target_link_libraries(speak_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${PYTHON_LIBRARIES}
  pybind11::embed
)

ament_target_dependencies(speak_node
  rclcpp
  std_msgs
)


# Install targets
install(TARGETS
  speak_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
