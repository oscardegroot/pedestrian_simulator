cmake_minimum_required(VERSION 3.0.2)
project(pedestrian_simulator)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(PEDSIM_SIMULATOR_DEPENDENCIES
geometry_msgs
roscpp
std_msgs
ros_tools
pedsim_original
lmpcc_msgs
asr_rapidxml
roslib
)

find_package(catkin REQUIRED COMPONENTS ${PEDSIM_SIMULATOR_DEPENDENCIES})

include_directories(
  include/pedestrian_simulator
  include/pedestrians
)

catkin_package(
  CATKIN_DEPENDS ${PEDSIM_SIMULATOR_DEPENDENCIES}
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES pedestrian_simulator
)

# ##########
# # Build ##
# ##########

include_directories(
  include
  include/pedestrian_simulator
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(pedestrian_simulator
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
  src/pedestrians/waypoint_pedestrian.cpp
)
add_dependencies(pedestrian_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pedestrian_simulator_node src/pedestrian_simulator_node.cpp)
target_link_libraries(pedestrian_simulator_node pedestrian_simulator ${catkin_LIBRARIES})
add_dependencies(pedestrian_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ############
# # Install ##
# ############
install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")
