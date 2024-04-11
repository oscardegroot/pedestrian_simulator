cmake_minimum_required(VERSION 3.8)
project(pedestrian_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# General requirements
set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  derived_object_msgs
  mpc_planner_msgs
  geometry_msgs
  ros_tools
  pedsim_original
  asr_rapidxml
  tf2_ros
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

## BUILD ###
include_directories(
  include
)

add_library(pedestrians SHARED
  src/pedestrians/binomial_pedestrian.cpp
  src/pedestrians/gaussian_pedestrian.cpp
  src/pedestrians/pedestrian.cpp
  src/pedestrians/random_gaussian_pedestrian.cpp
  src/pedestrians/social_forces_pedestrian.cpp
)
target_include_directories(pedestrians PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(pedestrians Eigen3::Eigen)
ament_target_dependencies(pedestrians
  ros_tools
)

add_library(${PROJECT_NAME} SHARED
  src/pedestrian_simulator.cpp
  src/pedsim_manager.cpp
  src/xml_reader.cpp
  src/spawn_randomizer.cpp
  src/configuration.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} pedestrians Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(${PROJECT_NAME}_node src/ros2_pedestrian_simulator.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## INSTALL ##
install(
  TARGETS 
  pedestrians
  ${PROJECT_NAME}
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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
