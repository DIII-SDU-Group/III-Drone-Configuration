cmake_minimum_required(VERSION 3.8)
project(iii_drone_configuration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

# Includes and link:
set(III_DRONE_CONFIGURATION_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include/"
)

include_directories(src/matlab_MPC_5Hz_hp10)
link_directories(src/matlab_MPC_5Hz_hp10)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(iii_drone_interfaces REQUIRED)

# Yaml cpp
# Prefer system package to avoid building vendored yaml-cpp and its deprecation warnings.
find_package(yaml-cpp QUIET CONFIG)
if(NOT yaml-cpp_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    yaml_cpp
    GIT_REPOSITORY https://github.com/DIII-SDU-Group/yaml-cpp.git
    GIT_TAG        0.8.0
  )

  FetchContent_GetProperties(yaml_cpp)

  if(NOT yaml_cpp_POPULATED)
    message(STATUS "Fetching yaml-cpp...")
    FetchContent_Populate(yaml_cpp)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Disable yaml-cpp tests" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "Disable yaml-cpp tools" FORCE)
    set(YAML_BUILD_SHARED_LIBS ON CACHE BOOL "Build shared yaml-cpp library" FORCE)
    set(_old_cmake_warn_deprecated ${CMAKE_WARN_DEPRECATED})
    set(CMAKE_WARN_DEPRECATED OFF)
    add_subdirectory(${yaml_cpp_SOURCE_DIR} ${yaml_cpp_BINARY_DIR})
    set(CMAKE_WARN_DEPRECATED ${_old_cmake_warn_deprecated})
  endif()
endif()

# Library files:
set(III_DRONE_CONFIGURATION_LIBRARY_HEADERS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/parameter_bundle.hpp"
)

set(III_DRONE_CONFIGURATION_LIBRARY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/parameter_bundle.cpp"
)

include_directories(
  ${III_DRONE_CONFIGURATION_INCLUDE_DIRS}
)

# Library:
add_library(iii_drone_configuration_lib 
  SHARED
  ${III_DRONE_CONFIGURATION_LIBRARY_HEADERS}
  ${III_DRONE_CONFIGURATION_LIBRARY_SOURCES}
)

target_sources(iii_drone_configuration_lib
  PUBLIC
  ${III_DRONE_CONFIGURATION_LIBRARY_HEADERS}
  ${III_DRONE_CONFIGURATION_LIBRARY_SOURCES}
)

target_include_directories(iii_drone_configuration_lib
  PUBLIC
  ${III_DRONE_CONFIGURATION_INCLUDE_DIRS}
)

target_link_libraries(iii_drone_configuration_lib
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(iii_drone_configuration_lib
  rclcpp
  rclcpp_lifecycle
  iii_drone_interfaces
)

ament_export_libraries(iii_drone_configuration_lib)
ament_export_include_directories(${III_DRONE_CONFIGURATION_INCLUDE_DIRS})

install(TARGETS iii_drone_configuration_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
# install(PROGRAMS
#   "${PROJECT_SOURCE_DIR}/scripts/configuration_server_node/configuration_server_node.py"
#   "${PROJECT_SOURCE_DIR}/scripts/configuration_client_node/configuration_client_node.py"
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime)
endif()

# Install header files
install(DIRECTORY 
  "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

ament_package()

set(python_programs
  "${PROJECT_NAME}/configuration_server_node.py"
  "${PROJECT_NAME}/configuration_client_node.py"
)

install(
  PROGRAMS ${python_programs}
  DESTINATION lib/${PROJECT_NAME}
)
