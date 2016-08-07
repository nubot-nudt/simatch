cmake_minimum_required(VERSION 2.8.3)
project(auto_referee)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nubot_common
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    roscpp
)				

add_executable(auto_referee src/auto_referee.cpp)
target_link_libraries(auto_referee ${catkin_LIBRARIES})