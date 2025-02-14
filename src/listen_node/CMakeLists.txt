cmake_minimum_required(VERSION 3.8)
project(listen_node)

# Set RPATH policies
cmake_policy(SET CMP0068 NEW)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)  # Crucial for portable builds

# Whisper configuration
set(WHISPER_CPP_DIR "/usr/local/src/whisper.cpp")
set(WHISPER_CPP_BUILD_DIR "${WHISPER_CPP_DIR}/build")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
    ${WHISPER_CPP_DIR}/include
    ${WHISPER_CPP_DIR}/ggml/include
)

# Add executable (remove unnecessary source files)
add_executable(listener_node 
    src/listener_node.cpp
)

# Link required libraries
target_link_libraries(listener_node
    ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so.1
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml-cpu.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml-base.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/ggml-cuda/libggml-cuda.so
)

# Configure RPATH settings
set_target_properties(listener_node PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib;${WHISPER_CPP_BUILD_DIR}/src"
    BUILD_WITH_INSTALL_RPATH FALSE
    SKIP_BUILD_RPATH FALSE
)

# Ament dependencies
ament_target_dependencies(listener_node 
    rclcpp 
    std_msgs
)

# Installation configuration
install(TARGETS listener_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(FILES
    ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so
    ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so.1
    ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so.1.7.4
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml-cpu.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/libggml-base.so
    ${WHISPER_CPP_BUILD_DIR}/ggml/src/ggml-cuda/libggml-cuda.so
    DESTINATION lib
)


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
