cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# enable Find*.cmake to be found
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)
SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

add_library(utils
  src/stability.cpp
  src/rbdlWrapper.cpp
)

target_include_directories(utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIRA}/include>
    $<INSTALL_INTERFACE:include>
    ${RBDL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# link dependencies
target_link_libraries(utils
  Eigen3::Eigen
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  nlopt
  m
)

ament_target_dependencies(utils
  RBDL
)

# install project launch files
install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install libraries
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS utils
  EXPORT export_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()