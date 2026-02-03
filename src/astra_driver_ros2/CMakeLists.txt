cmake_minimum_required(VERSION 3.8)
project(astra_driver_ros2)

# --------------------
# Compiler options
# --------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --------------------
# OpenNI2 SDK paths (CORRECT)
# --------------------
set(OPENNI2_ROOT
  /home/anup/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux
)

set(OPENNI2_INCLUDE_DIR
  ${OPENNI2_ROOT}/sdk/Include
)

set(OPENNI2_LIBRARY
  ${OPENNI2_ROOT}/sdk/libs/libOpenNI2.so
)

# --------------------
# Include directories
# --------------------
include_directories(
  ${OPENNI2_INCLUDE_DIR}
)

# --------------------
# Build executable
# --------------------
add_executable(astra_node
  src/astra_node.cpp
)

# --------------------
# Link libraries
# --------------------
target_link_libraries(astra_node
  ${OPENNI2_LIBRARY}
)

ament_target_dependencies(astra_node
  rclcpp
  sensor_msgs
  std_msgs
)

# --------------------
# RPATH (CRITICAL)
# --------------------
set_target_properties(astra_node PROPERTIES
  BUILD_RPATH ${OPENNI2_ROOT}/sdk/libs
  INSTALL_RPATH ${OPENNI2_ROOT}/sdk/libs
)

# --------------------
# Install
# --------------------
install(
  TARGETS astra_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

