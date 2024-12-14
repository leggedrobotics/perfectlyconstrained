# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
#add_compile_options(-Wall -Wextra -Wpedantic)

#add_definitions(-mno-avx -mno-sse2 -fno-math-errno -ffast-math)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
  # add_definitions(-O3 -msse2 -msse3 -msse4 -mavx -mfma)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  libnabo
  message_logger
  qpmad
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(Boost REQUIRED COMPONENTS chrono date_time filesystem program_options system thread timer)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
  LIBRARIES
    yaml_cpp_pm
    pointmatcher
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

########################
## Library definition ##
########################
# Yaml-cpp
file(GLOB YAML_PRIVATE_HEADERS "contrib/yaml-cpp-pm/src/[a-zA-Z]*.h")
file(GLOB YAML_SOURCES "contrib/yaml-cpp-pm/src/[a-zA-Z]*.cpp")
add_library(yaml_cpp_pm
  ${YAML_SOURCES}
  ${YAML_PRIVATE_HEADERS}
)
target_include_directories(yaml_cpp_pm PUBLIC
  contrib/yaml-cpp-pm/include
)
target_include_directories(yaml_cpp_pm PRIVATE
  contrib/yaml-cpp-pm/src
)
target_compile_options(yaml_cpp_pm PRIVATE -w)

# Libpointmatcher
add_library(pointmatcher
  ${POINTMATCHER_SRC}
  ${POINTMATCHER_HEADERS}
)
add_dependencies(pointmatcher
  yaml_cpp_pm
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(pointmatcher PUBLIC
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/pointmatcher
  ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
  ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
)
target_include_directories(pointmatcher SYSTEM PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_pm_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
target_link_libraries(pointmatcher
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  Boost::chrono
  Boost::date_time
  Boost::filesystem
  Boost::program_options
  Boost::thread
  Boost::timer
  Boost::system
  yaml_cpp_pm
)

#############
## Install ##
#############
install(
  TARGETS
    pointmatcher
    yaml_cpp_pm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    ${CMAKE_SOURCE_DIR}/pointmatcher/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/pointmatcher
)

##########
## Test ##
##########
find_package(cmake_code_coverage QUIET)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pointmatcher
      utest/utest.cpp
      utest/ui/DataFilters.cpp
      utest/ui/DataPoints.cpp
      utest/ui/ErrorMinimizers.cpp
      utest/ui/Inspectors.cpp
      utest/ui/IO.cpp
      utest/ui/Loggers.cpp
      utest/ui/Matcher.cpp
      utest/ui/Outliers.cpp
      utest/ui/Transformations.cpp
      utest/ui/octree/Octree.cpp
      utest/ui/localizability/PointContributionLocalizabilityDetection.cpp
      utest/ui/localizability/EqualityConstrainedOptimization.cpp
      utest/ui/localizability/InequalityConstrainedOptimization.cpp
      utest/ui/localizability/SolutionRemapping.cpp
  )
  add_dependencies(test_pointmatcher
    pointmatcher
    yaml_cpp_pm
  )
  target_include_directories(test_pointmatcher PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pointmatcher
    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters
    ${CMAKE_SOURCE_DIR}/pointmatcher/DataPointsFilters/utils
  )
  target_include_directories(test_pointmatcher SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_pointmatcher
    pointmatcher
    yaml_cpp_pm
    ${catkin_LIBRARIES}
  )

  set(TEST_DATA_FOLDER "${CMAKE_SOURCE_DIR}/examples/data/")
  add_definitions(-DUTEST_TEST_DATA_PATH="${TEST_DATA_FOLDER}/")

  ##################
  # Code_coverage ##
  ##################
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_pointmatcher
      SOURCE_EXCLUDE_PATTERN
        ${PROJECT_SOURCE_DIR}/contrib/yaml-cpp-pm/include/yaml-cpp-pm/** # Cold copy of yaml-cpp headers.
        ${PROJECT_SOURCE_DIR}/contrib/yaml-cpp-pm/src/** # Cold copy of yaml-cpp sources.
        ${PROJECT_SOURCE_DIR}/utest/*
        ${PROJECT_SOURCE_DIR}/utest/ui/*
    )
  endif()
endif(CATKIN_ENABLE_TESTING)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)