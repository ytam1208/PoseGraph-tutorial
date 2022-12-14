cmake_minimum_required(VERSION 3.0.2)
project(Graph_tuto)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES Graph_tuto
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
if (BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Graph_tuto.cpp
# )

add_executable(${PROJECT_NAME}_node  src/DataBase.cpp src/ICP.cpp src/kdtree.cpp src/Solver.cpp src/Graph.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
 ${CERES_LIBRARIES}
)
