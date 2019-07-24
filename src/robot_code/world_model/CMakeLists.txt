cmake_minimum_required(VERSION 2.8.3)
project(world_model)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")
#if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
#else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_BUILD_TYPE "Release")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
#endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nubot_common
  dynamic_reconfigure
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads)

set(world_model_includes
               ${catkin_INCLUDE_DIRS}
               ${Boost_INCLUDE_DIRS}
               ${EIGEN_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${world_model_includes})

add_library(world_model  STATIC
                         src/ball.cpp
                         src/obstacles.cpp
                         src/robot.cpp
                         src/MTTracker.cpp
                         src/iAuction.cpp
)
add_dependencies(world_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(world_model
    ${catkin_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS   include
  CATKIN_DEPENDS dynamic_reconfigure
                 message_filters
                 message_generation
                 roscpp
                 rospy
                 std_msgs
                 std_srvs
  LIBRARIES      world_model
)

add_subdirectory(rtdb)
add_executable(world_model_node
               src/main.cpp
               src/world_model.cpp
)
target_link_libraries(world_model_node
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    world_model
    rtdb
    multicast
)
add_dependencies(world_model_node ${catkin_EXPORTED_TARGETS})


