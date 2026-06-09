cmake_minimum_required(VERSION 3.5)
project(pairs_uav_trackers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Wextra)

if(COVERAGE)
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
  pairs_uav_testing
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

generate_dynamic_reconfigure_options(
  cfg/mpc_tracker.cfg
  cfg/flip_tracker.cfg
  )

# Detect architecture for pre-compiled MPC solver binary
execute_process(COMMAND dpkg-architecture -qDEB_HOST_ARCH
  OUTPUT_VARIABLE CMAKE_DEB_HOST_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)

if(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64|x64")
  set(MPC_TRACKER_SOLVER_BIN
    ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/x64/libMpcTrackerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(MPC_TRACKER_SOLVER_BIN
    ${PROJECT_SOURCE_DIR}/lib/MpcTrackerSolver/arm64/libMpcTrackerSolver.so)
else()
  message(FATAL_ERROR "MpcTrackerSolver.so not available for ${CMAKE_DEB_HOST_ARCH}")
endif()

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(PairsUavTrackers_MpcTracker src/mpc_tracker/mpc_tracker.cpp)
add_dependencies(PairsUavTrackers_MpcTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_MpcTracker
  ${catkin_LIBRARIES} ${MPC_TRACKER_SOLVER_BIN})

add_library(PairsUavTrackers_LineTracker src/line_tracker/line_tracker.cpp)
add_dependencies(PairsUavTrackers_LineTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_LineTracker ${catkin_LIBRARIES})

add_library(PairsUavTrackers_LandoffTracker src/landoff_tracker/landoff_tracker.cpp)
add_dependencies(PairsUavTrackers_LandoffTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_LandoffTracker ${catkin_LIBRARIES})

add_library(PairsUavTrackers_JoyTracker src/joy_tracker/joy_tracker.cpp)
add_dependencies(PairsUavTrackers_JoyTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_JoyTracker ${catkin_LIBRARIES})

add_library(PairsUavTrackers_SpeedTracker src/speed_tracker/speed_tracker.cpp)
add_dependencies(PairsUavTrackers_SpeedTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_SpeedTracker ${catkin_LIBRARIES})

add_library(PairsUavTrackers_FlipTracker src/flip_tracker/flip_tracker.cpp)
add_dependencies(PairsUavTrackers_FlipTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_FlipTracker ${catkin_LIBRARIES})

add_library(PairsUavTrackers_MidairActivationTracker
  src/midair_activation_tracker/midair_activation_tracker.cpp)
add_dependencies(PairsUavTrackers_MidairActivationTracker
  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(PairsUavTrackers_MidairActivationTracker ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING AND PAIRS_ENABLE_TESTING)
  message(WARNING "Testing enabled.")
  add_subdirectory(test)
endif()

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
