cmake_minimum_required(VERSION 2.8.3)
project(semantic_route_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ontologenius
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES semantic_route_description
#  CATKIN_DEPENDS ontologenius roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(semantic_finder_lib
  src/OntologyManipulator.cpp
  src/RegionPathfinder.cpp
  src/CorridorPathfinder.cpp
  src/PlacePathfinder.cpp
  src/PlaceToRegion.cpp
)

add_executable(description src/main.cpp)
 target_link_libraries(description
   ${catkin_LIBRARIES}
 )
 add_dependencies(description ${catkin_EXPORTED_TARGETS})
 target_link_libraries(description semantic_finder_lib)
