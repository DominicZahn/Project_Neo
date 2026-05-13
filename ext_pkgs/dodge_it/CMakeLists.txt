cmake_minimum_required(VERSION 3.8)
project(dodge_it)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
list(APPEND CMAKE_PREFIX_PATH ${PARENT_DIR}/CMakeModules)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(neo_utils REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)
set(NLopt_DIR /opt/venv/lib/cmake/nlopt)
find_package(NLopt REQUIRED)

add_executable(smith src/smith.cpp)
target_include_directories(smith PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# install project launch files and configs
install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

target_link_libraries(smith
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  NLopt::nlopt
  m
)

ament_target_dependencies(
  smith
  rclcpp
  RBDL
  neo_utils
)

target_compile_features(smith PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS smith
  DESTINATION lib/${PROJECT_NAME})

ament_package()
