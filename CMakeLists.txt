cmake_minimum_required(VERSION 3.5)
project(pairs_uav_trackers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  rospy
  geometry_msgs
  tf
  std_msgs
  pairs_lib
  pairs_msgs
  pairs_uav_managers
  dynamic_reconfigure
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/mpc_tracker.cfg
  cfg/flip_tracker.cfg
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

set(LIBRARIES
  PairsUavTrackers_MpcTracker
  PairsUavTrackers_LineTracker
  PairsUavTrackers_LandoffTracker
  PairsUavTrackers_JoyTracker
  PairsUavTrackers_SpeedTracker
  PairsUavTrackers_FlipTracker
  PairsUavTrackers_MidairActivationTracker
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Mpc Solver Library

# Store in CMAKE_DEB_HOST_ARCH var the current build architecture
execute_process(COMMAND
  dpkg-architecture
  -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
  CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

# deduce the library path based on the system architecture
if(${CMAKE_DEB_HOST_ARCH} MATCHES "armhf")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for armhf")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  MESSAGE(FATAL_ERROR "Mising MpcTrackerSolver.so for i386")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/x64/libMpcTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/x64/libMpcTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/arm64/libMpcTrackerSolver.so)
else()
  MESSAGE(FATAL_ERROR ${CMAKE_DEB_HOST_ARCH})
  MESSAGE(FATAL_ERROR "MpcTrackerSolver.so has not been selected, check CMakeLists.txt.")
endif()

# MPC Tracker

add_library(PairsUavTrackers_MpcTracker
  src/mpc_tracker/mpc_tracker.cpp
  )

add_dependencies(PairsUavTrackers_MpcTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_MpcTracker
  ${catkin_LIBRARIES}
  ${MPC_CONTROLLER_SOLVER_BIN}
  )

# Line Tracker

add_library(PairsUavTrackers_LineTracker
  src/line_tracker/line_tracker.cpp
  )

add_dependencies(PairsUavTrackers_LineTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_LineTracker
  ${catkin_LIBRARIES}
  )

# Landoff Tracker

add_library(PairsUavTrackers_LandoffTracker
  src/landoff_tracker/landoff_tracker.cpp
  )

add_dependencies(PairsUavTrackers_LandoffTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_LandoffTracker
  ${catkin_LIBRARIES}
  )

# Joy Tracker

add_library(PairsUavTrackers_JoyTracker
  src/joy_tracker/joy_tracker.cpp
  )

add_dependencies(PairsUavTrackers_JoyTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_JoyTracker
  ${catkin_LIBRARIES}
  )

# Speed tracker

add_library(PairsUavTrackers_SpeedTracker
  src/speed_tracker/speed_tracker.cpp
  )

add_dependencies(PairsUavTrackers_SpeedTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_SpeedTracker
  ${catkin_LIBRARIES}
  )

# Flip tracker

add_library(PairsUavTrackers_FlipTracker
  src/flip_tracker/flip_tracker.cpp
  )

add_dependencies(PairsUavTrackers_FlipTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_FlipTracker
  ${catkin_LIBRARIES}
  )

# Midair activation tracker

add_library(PairsUavTrackers_MidairActivationTracker
  src/midair_activation_tracker/midair_activation_tracker.cpp
  )

add_dependencies(PairsUavTrackers_MidairActivationTracker
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(PairsUavTrackers_MidairActivationTracker
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(CATKIN_ENABLE_TESTING AND PAIRS_ENABLE_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES ${MPC_CONTROLLER_SOLVER_BIN}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
