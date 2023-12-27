cmake_minimum_required(VERSION 3.10)
project(ceres_tutorial)

include_directories("/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
 include
 ${CERES_INCLUDE_DIRS}
)

# Should be larger than version 14 to avoid ‘integer_sequence’ is not a member of ‘std’ error
set(CMAKE_CXX_STANDARD 17)

# Only Ceres itself should be compiled with CERES_BUILDING_SHARED_LIBRARY
# defined, any users of Ceres will have CERES_USING_SHARED_LIBRARY defined
# for them in Ceres' config.h if appropriate.
if (BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()

add_executable(ceres_tutorial pose_graph_2d.cc)
target_link_libraries(ceres_tutorial ceres)

