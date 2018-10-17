cmake_minimum_required(VERSION 2.8.3)
project(hdl_graph_slam_reconstruction)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions("-g")

SET(CMAKE_CXX_FLAGS "-std=c++11 -g -O2 -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy tf message_generation message_runtime
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##########################
### Message generation ###
##########################
add_message_files(
  FILES
  FloorCoeffs1.msg
)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hdl_graph_slam_reconstruction
  DEPENDS message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
 include
 ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

add_executable(prefiltering
    src/prefiltering.cpp)
add_dependencies(prefiltering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(prefiltering
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


add_executable(scanMatchingOdometry
  src/scanMatchingOdometry.cpp
)
add_dependencies(scanMatchingOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scanMatchingOdometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(floorDetection
  src/floorDetection.cpp
)
add_dependencies(floorDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(floorDetection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(loopDetection
  src/loopDetection.cpp
)
add_dependencies(loopDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(loopDetection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)