cmake_minimum_required(VERSION 3.8)
project(mover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# enable Find*.cmake to be found
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# find dependencies
SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(Eigen3 3.0.0 REQUIRED)

add_executable(staticJointMover src/staticJointMover.cpp)
target_include_directories(staticJointMover PUBLIC ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

target_link_libraries(staticJointMover
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  nlopt
  m
)
ament_target_dependencies(
  staticJointMover
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  RBDL
)

install(TARGETS staticJointMover
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
