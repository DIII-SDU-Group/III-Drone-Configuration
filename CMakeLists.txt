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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

find_package(pybind11 QUIET CONFIG)
if(NOT pybind11_FOUND)
  find_package(pybind11_vendor QUIET)
  find_package(pybind11 QUIET CONFIG)
endif()
if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.13.6
  )
  FetchContent_MakeAvailable(pybind11)
endif()

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

if(TARGET yaml-cpp::yaml-cpp)
  set(III_DRONE_CONFIGURATION_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(III_DRONE_CONFIGURATION_YAML_CPP_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp target not found")
endif()

# Library files:
set(III_DRONE_CONFIGURATION_LIBRARY_HEADERS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/configurator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/configuration.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/python_core.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_configuration/schema_validator.hpp"
)

set(III_DRONE_CONFIGURATION_LIBRARY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/configurator.cpp"
  "${PROJECT_SOURCE_DIR}/src/configuration.cpp"
  "${PROJECT_SOURCE_DIR}/src/python_core.cpp"
  "${PROJECT_SOURCE_DIR}/src/schema_validator.cpp"
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
  ${III_DRONE_CONFIGURATION_YAML_CPP_TARGET}
)

ament_target_dependencies(iii_drone_configuration_lib
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_msgs
  iii_drone_interfaces
)

ament_export_libraries(iii_drone_configuration_lib)
ament_export_include_directories(${III_DRONE_CONFIGURATION_INCLUDE_DIRS})

pybind11_add_module(_native MODULE
  "${PROJECT_SOURCE_DIR}/src/python_bindings.cpp"
)
set_target_properties(_native PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
)
target_link_libraries(_native
  PRIVATE
  iii_drone_configuration_lib
)
target_include_directories(_native
  PRIVATE
  ${III_DRONE_CONFIGURATION_INCLUDE_DIRS}
)

install(TARGETS iii_drone_configuration_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS _native
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
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

set(python_entrypoint_wrappers
  "${PROJECT_SOURCE_DIR}/scripts/configuration_server"
  "${PROJECT_SOURCE_DIR}/scripts/configuration_client"
)

install(
  PROGRAMS ${python_programs}
           ${python_entrypoint_wrappers}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  set(III_DRONE_CONFIGURATION_TEST_SCHEMA_FILE
    "${PROJECT_SOURCE_DIR}/test/resources/test_parameter_manifest.yaml"
  )
  set(III_DRONE_CONFIGURATION_PRODUCTION_SCHEMA_FILE
    "${PROJECT_SOURCE_DIR}/config/parameters/parameter_manifest.yaml"
  )
  set(III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_REAL_FILE
    "${PROJECT_SOURCE_DIR}/config/parameter_sets/real/tracked/default.yaml"
  )
  set(III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_SIM_FILE
    "${PROJECT_SOURCE_DIR}/config/parameter_sets/sim/tracked/default.yaml"
  )

  ament_add_gtest(test_schema_validator
    "${PROJECT_SOURCE_DIR}/test/test_schema_validator.cpp"
  )
  target_link_libraries(test_schema_validator iii_drone_configuration_lib)
  target_compile_definitions(test_schema_validator PRIVATE
    TEST_SCHEMA_FILE="${III_DRONE_CONFIGURATION_TEST_SCHEMA_FILE}"
    PRODUCTION_SCHEMA_FILE="${III_DRONE_CONFIGURATION_PRODUCTION_SCHEMA_FILE}"
    PRODUCTION_ROS_PARAMS_REAL_FILE="${III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_REAL_FILE}"
    PRODUCTION_ROS_PARAMS_SIM_FILE="${III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_SIM_FILE}"
  )
  ament_target_dependencies(test_schema_validator rclcpp)

  ament_add_gtest(test_configuration
    "${PROJECT_SOURCE_DIR}/test/test_configuration.cpp"
  )
  target_link_libraries(test_configuration iii_drone_configuration_lib)
  ament_target_dependencies(test_configuration rclcpp)

  ament_add_gtest(test_configurator
    "${PROJECT_SOURCE_DIR}/test/test_configurator.cpp"
  )
  target_link_libraries(test_configurator iii_drone_configuration_lib)
  target_compile_definitions(test_configurator PRIVATE
    TEST_SCHEMA_FILE="${III_DRONE_CONFIGURATION_TEST_SCHEMA_FILE}"
    PRODUCTION_SCHEMA_FILE="${III_DRONE_CONFIGURATION_PRODUCTION_SCHEMA_FILE}"
  )
  ament_target_dependencies(test_configurator rclcpp rclcpp_lifecycle)

  ament_add_gtest(test_python_core
    "${PROJECT_SOURCE_DIR}/test/test_python_core.cpp"
  )
  target_link_libraries(test_python_core iii_drone_configuration_lib)
  target_compile_definitions(test_python_core PRIVATE
    TEST_SCHEMA_FILE="${III_DRONE_CONFIGURATION_TEST_SCHEMA_FILE}"
  )
  ament_target_dependencies(test_python_core rclcpp)

  add_executable(configurator_managed_test_node
    "${PROJECT_SOURCE_DIR}/test/configurator_managed_test_node.cpp"
  )
  target_link_libraries(configurator_managed_test_node iii_drone_configuration_lib)
  target_compile_definitions(configurator_managed_test_node PRIVATE
    TEST_SCHEMA_FILE="${III_DRONE_CONFIGURATION_TEST_SCHEMA_FILE}"
  )
  ament_target_dependencies(configurator_managed_test_node rclcpp rclcpp_lifecycle)

  ament_add_pytest_test(
    test_python_schema_utils
    "${PROJECT_SOURCE_DIR}/test/test_schema_utils.py"
    APPEND_ENV
      PYTHONPATH=${PROJECT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}
  )

  ament_add_pytest_test(
    test_python_configuration
    "${PROJECT_SOURCE_DIR}/test/test_python_configuration.py"
    APPEND_ENV
      PYTHONPATH=${PROJECT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}
  )

  ament_add_pytest_test(
    test_python_parameter_handler
    "${PROJECT_SOURCE_DIR}/test/test_parameter_handler.py"
    APPEND_ENV
      PYTHONPATH=${PROJECT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}
    ENV
      III_DRONE_PRODUCTION_SCHEMA_FILE=${III_DRONE_CONFIGURATION_PRODUCTION_SCHEMA_FILE}
      III_DRONE_PRODUCTION_ROS_PARAMS_REAL_FILE=${III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_REAL_FILE}
      III_DRONE_PRODUCTION_ROS_PARAMS_SIM_FILE=${III_DRONE_CONFIGURATION_PRODUCTION_ROS_PARAMS_SIM_FILE}
  )

  ament_add_pytest_test(
    test_python_configuration_server
    "${PROJECT_SOURCE_DIR}/test/test_configuration_server.py"
    TIMEOUT 120
    APPEND_ENV
      PYTHONPATH=${PROJECT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}
  )

  ament_add_pytest_test(
    test_python_configuration_e2e
    "${PROJECT_SOURCE_DIR}/test/test_configuration_e2e.py"
    APPEND_ENV
      PYTHONPATH=${PROJECT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}:$ENV{PYTHONPATH}
    ENV
      ROS_DOMAIN_ID=221
  )

  ament_add_pytest_test(
    test_python_install_layout
    "${PROJECT_SOURCE_DIR}/test/test_install_layout.py"
  )
endif()
