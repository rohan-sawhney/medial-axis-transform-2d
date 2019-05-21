cmake_minimum_required(VERSION 3.0)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake)
project(MedialAxisTransform2D)

###############################################################################
## file globbing ##############################################################
###############################################################################

# these instructions search the directory tree when cmake is
# invoked and put all files that match the pattern in the variables 
# `sources` and `data`
file(GLOB_RECURSE sources      medial-axis-transform-2d/*.cpp medial-axis-transform-2d/*.h)
file(GLOB_RECURSE sources_test medial-axis-transform-2d/test/*.cpp)
#file(GLOB_RECURSE data resources/*)


###############################################################################
## target definitions #########################################################
###############################################################################

# add the data to the target, so it becomes visible in some IDE
add_executable(MedialAxisTransform2D ${sources})

# just for example add some compiler flags
target_compile_options(MedialAxisTransform2D PUBLIC -std=c++17 -Wall)

# this lets me include files relative to the root src dir with a <> pair
target_include_directories(MedialAxisTransform2D PUBLIC medial-axis-transform-2d)

###############################################################################
## dependencies ###############################################################
###############################################################################

# this defines the variables Boost_LIBRARIES that contain all library names
# that we need to link to
#find_package(Stb REQUIRED)
#include_directories(${Stb_INCLUDE_DIRS})
#include_directories(${CMAKE_CURRENT_BINARY_DIR} libs)
#add_dependencies(MedialAxisTransform2D stb)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (MedialAxisTransform2D Eigen3::Eigen)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

target_link_libraries(MedialAxisTransform2D ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

###############################################################################
## testing ####################################################################
###############################################################################

# this is for our testing framework
# we don't add REQUIRED because it's just for testing
#find_package(GTest)

if(GTEST_FOUND)
  add_executable(unit_tests ${sources_test} ${sources})

  # we add this define to prevent collision with the main
  # this might be better solved by not adding the source with the main to the
  # testing target
  target_compile_definitions(unit_tests PUBLIC UNIT_TESTS)

  # this allows us to use our executable as a link library
  # therefore we can inherit all compiler options and library dependencies
  set_target_properties(MedialAxisTransform2D PROPERTIES ENABLE_EXPORTS on)

  
  target_link_libraries(unit_tests PUBLIC
    ${GTEST_BOTH_LIBRARIES}
    MedialAxisTransform2D
  )

  target_include_directories(unit_tests PUBLIC
    ${GTEST_INCLUDE_DIRS} # doesn't do anything on Linux
  )
endif()

###############################################################################
## packaging ##################################################################
###############################################################################

# all install commands get the same destination. this allows us to use paths
# relative to the executable.
install(TARGETS MedialAxisTransform2D DESTINATION example_destination)
# this is basically a repeat of the file copy instruction that copies the
# resources in the build directory, but here we tell cmake that we want it
# in the package
install(DIRECTORY resources DESTINATION example_destination)

# now comes everything we need, to create a package
# there are a lot more variables you can set, and some
# you need to set for some package types, but we want to
# be minimal here
set(CPACK_PACKAGE_NAME "MedialAxisTransform2D")
set(CPACK_PACKAGE_VERSION "1.0.0")

# we don't want to split our program up into several things
set(CPACK_MONOLITHIC_INSTALL 1)

# This must be last
include(CPackConfig.cmake)
