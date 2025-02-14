cmake_minimum_required(VERSION 3.8)
project(listen_node)

# Add the path to the cmake/ folder in whisper.cpp
set(WHISPER_CPP_DIR "/usr/local/src/whisper.cpp")
set(CMAKE_MODULE_PATH "${WHISPER_CPP_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Path to the whisper.cpp build directory where libwhisper.a is generated
set(WHISPER_CPP_BUILD_DIR "${WHISPER_CPP_DIR}/build")

# Add whisper.cpp source files
set(WHISPER_CPP_SOURCES
    ${WHISPER_CPP_DIR}/src/whisper.cpp  # Adjusted to point to the actual file
    ${WHISPER_CPP_DIR}/ggml/     # Include ggml core if required
    ${WHISPER_CPP_DIR}/ggml/include     # Include ggml include manually if required
    ${WHISPER_CPP_DIR}/cmake     # Include cmake after building if required
)


# Ensure CMake knows about this path
link_directories(${WHISPER_CPP_BUILD_DIR})


# Add include directories
include_directories(
    ${WHISPER_CPP_DIR}/include
    ${WHISPER_CPP_DIR}/ggml
    ${WHISPER_CPP_DIR}/ggml/include     # Include ggml core if required
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(listener_node src/listener_node.cpp ${WHISPER_CPP_DIR}/src)

# Link whisper library
target_link_libraries(listener_node ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so)  # Link the static library

ament_target_dependencies(listener_node rclcpp std_msgs)

install(TARGETS listener_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
