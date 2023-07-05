cmake_minimum_required(VERSION 3.0.2)
project(pedestrian_simulator)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-fstack-protector)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  ros_tools
  pedsim_original
  lmpcc_msgs
  asr_rapidxml
  roslib
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
include_directories(
  include/pedestrian_simulator
  include/pedestrians
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs asr_rapidxml roslib pedsim_original lmpcc_msgs
  INCLUDE_DIRS include include/${PROJECT_NAME}
  LIBRARIES pedestrian_simulator
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
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

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# ${catkin_LIBRARIES}
# )

# ############
# # Install ##
# ############
install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")
