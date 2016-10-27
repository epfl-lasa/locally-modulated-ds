cmake_minimum_required(VERSION 2.8.3)
project(locally_modulated_ds)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gaussian_process_regression)



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES locally_modulated_ds
CATKIN_DEPENDS gaussian_process_regression
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
FILE(GLOB theIncludeFiles include/*.h)
#add_library(gazebo_particle_filter_plugin ${theSourceFiles} ${theIncludeFiles})

## Declare a cpp library
add_library(snippet_gpmds
        src/snippet_gpmds.cpp
${theIncludeFiles}
)


add_executable(run_snippet_gpmds
        src/run_snippet_gpmds.cpp
)
target_link_libraries(run_snippet_gpmds snippet_gpmds)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(lmds-test test/test_locally_modulated_ds.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
catkin_add_gtest(gpmds-test test/test_gp_modulated_ds.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
catkin_add_gtest(snippet_gpmds-test test/test_snippet_gpmds.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(snippet_gpmds-test snippet_gpmds)
catkin_add_gtest(snippet-test test/test_snippet.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(snippet-test)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
