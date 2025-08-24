cmake_minimum_required(VERSION 3.8)
project(neo_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(tinyxml2 REQUIRED)
# enable Find*.cmake to be found
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

add_library(neo_utils
  src/stability.cpp
  src/rbdlWrapper.cpp
)

target_include_directories(neo_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${RBDL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# link dependencies
target_link_libraries(neo_utils
  Eigen3::Eigen
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  tinyxml2::tinyxml2
)

ament_target_dependencies(
  neo_utils
  rclcpp
  sensor_msgs
  ros_gz_interfaces
  RBDL
  tinyxml2
)  

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS neo_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  RBDL
  rclcpp
  sensor_msgs
  ros_gz_interfaces
  RBDL
)

ament_package()