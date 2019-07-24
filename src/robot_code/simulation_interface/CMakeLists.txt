cmake_minimum_required(VERSION 2.8.3)
project(simulation_interface)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nubot_common
  message_filters
  message_generation
  roscpp
  rospy
  world_model
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads)

set(simulation_interface_includes
               include
               ${catkin_INCLUDE_DIRS}
               ${Boost_INCLUDE_DIRS}
               ${EIGEN_INCLUDE_DIRS}
               ../world_model/include
)

include_directories(${simulation_interface_includes})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_filters
    message_generation
    roscpp
    rospy
)

### coach_robot_comm_RTDB_node is for simulation with RTDB; not used now
# add_executable(coach_robot_comm_RTDB_node  src/coach_robot_comm_RTDB.cpp)
# target_link_libraries(coach_robot_comm_RTDB_node  ${catkin_LIBRARIES}
#                                                  rtdb
#                                                  multicast
#                                                  world_model)

add_executable(strategy_pub_node src/strategy_info.cpp)
add_dependencies(strategy_pub_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(strategy_pub_node ${catkin_LIBRARIES})

