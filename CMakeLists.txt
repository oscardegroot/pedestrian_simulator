cmake_minimum_required(VERSION 3.5)
project(pedestrian_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

## PACKAGES ##
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(derived_object_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_planner_msgs REQUIRED)

find_package(ros_tools REQUIRED)
find_package(pedsim_original REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

find_package(asr_rapidxml REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

## BUILD ###
include_directories(
  include/${PROJECT_NAME}
  include/pedestrians
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/pedestrian_simulator.cpp
  src/pedsim_manager.cpp
  src/xml_reader.cpp
  src/spawn_randomizer.cpp
  src/configuration.cpp

  # Pedestrians
  src/pedestrians/binomial_pedestrian.cpp
  src/pedestrians/gaussian_pedestrian.cpp
  src/pedestrians/pedestrian.cpp
  src/pedestrians/random_gaussian_pedestrian.cpp
  src/pedestrians/social_forces_pedestrian.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  autoware_auto_perception_msgs
  autoware_adapi_v1_msgs
  derived_object_msgs
  mpc_planner_msgs
  geometry_msgs
  ros_tools
  pedsim_original
  asr_rapidxml
  tf2_ros
)

add_executable(${PROJECT_NAME}_node src/ros2_pedestrian_simulator.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## INSTALL ##
# The library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
# Pedestrian scenarios
install(DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME})

## FINALIZE PACKAGE ##
# ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(autoware_auto_perception_msgs)
ament_export_dependencies(autoware_adapi_v1_msgs)
ament_export_dependencies(derived_object_msgs)
ament_export_dependencies(lmpcc_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(ros_tools)
ament_export_dependencies(pedsim_original)
ament_export_dependencies(asr_rapidxml)
ament_export_dependencies(tf2_ros)
ament_package()
