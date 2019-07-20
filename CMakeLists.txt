cmake_minimum_required(VERSION 2.8.3)
project(planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES planner
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set (SRCS src/main.cpp
          src/trajectory.cpp
          src/visualize.cpp
          src/planner.cpp
          src/obstacle.cpp
          src/transform.cpp
          # src/optimizer.cpp
          )

set (LIBS ${catkin_LIBRARIES}
          g2o_core
          g2o_stuff)

add_executable(${PROJECT_NAME}_node ${SRCS})
target_link_libraries(${PROJECT_NAME}_node ${LIBS})

