cmake_minimum_required(VERSION 2.8.3)
project(pcl_merge)

set(CMAKE_BUILD_TYPE Release)

message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_stl_containers
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
  message_filters
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs tf
  DEPENDS Boost EIGEN3 YAML_CPP tf
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)

add_executable(pcl_merge
    src/pcl_merge.cpp
)

target_link_libraries(pcl_merge
  ${catkin_LIBRARIES}
)

add_dependencies(pcl_merge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


