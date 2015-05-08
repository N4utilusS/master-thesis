cmake_minimum_required(VERSION 2.6)
project(frontal_barrier_lp)

# Check for Lua and add include path
find_package(Lua51 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

# Check for ARGoS3
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)

# Add ARGoS paths
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

# Create the loop function library
add_library(frontal_barrier_lp SHARED frontal_barrier_lp.h frontal_barrier_lp.cpp)
target_link_libraries(frontal_barrier_lp
  ${ARGOS_LIBRARIES}
  ${LUA_LIBRARIES}
  argos3plugin_simulator_entities)
