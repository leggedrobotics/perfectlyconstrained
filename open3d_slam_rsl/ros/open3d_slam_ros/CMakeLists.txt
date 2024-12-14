cmake_minimum_required(VERSION 3.5)
project(open3d_slam_ros)

set(CMAKE_CXX_STANDARD 17)

## Source: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
#add_compile_options(-Wall -Wextra -Wpedantic -O3)
add_compile_options(-O3)

#add_definitions(-mno-avx -mno-sse2 -fno-math-errno -ffast-math -fno-math-errno)
#add_compile_options(-mno-avx -mno-sse2 -fno-math-errno -ffast-math -fno-math-errno)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
  src/helpers_ros.cpp
  src/SlamMapInitializer.cpp
  src/SlamWrapperRos.cpp
  src/OnlineRangeDataProcessorRos.cpp
  src/creators.cpp
  src/DataProcessorRos.cpp
  src/RosbagRangeDataProcessorRos.cpp
  src/Color.cpp
  src/Gnss.cpp
  src/GnssHandler.cpp
  src/ImuBuffer.cpp
  )

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  open3d_conversions
  open3d_slam
  eigen_conversions
  nav_msgs
  geometry_msgs
  rosgraph_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag
  interactive_markers
  open3d_slam_lua_io
  rms
  )

# We pass the path of the `FindXYZ.cmake` files to `CMAKE_MODULE_PATH`
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

# You can individually give the path of the libraries as well.
#list(APPEND LIBRARIES ".....so.0")
#list(APPEND LIBRARIES ".......so")

#find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-Dopen3d_slam_ros_OPENMP_FOUND=${OpenMP_FOUND})
endif ()
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  )

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  yaml-cpp
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenMP_CXX_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIRS}
)

add_definitions(-std=c++17)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
  )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${gflags_LIBRARIES}
  ${GLOG_LIBRARIES}
  yaml-cpp
  ${OpenMP_CXX_LIBRARIES}
  )

add_executable(mapping_node
  src/mapping_node.cpp)

target_link_libraries(mapping_node
  ${catkin_LIBRARIES}
  ${gflags_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${PROJECT_NAME}
  )

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " mapping_node)
  add_clang_tooling(
    TARGETS mapping_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
    #CT_ATTACH_TO_ALL # Uncomment to attach to all targets during build, takes time
  )
endif(cmake_clang_tools_FOUND)

add_executable(adjacency_graph_example_node
  src/adjacency_graph_example_node.cpp
  )

target_link_libraries(adjacency_graph_example_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )


