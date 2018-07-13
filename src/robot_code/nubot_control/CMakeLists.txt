cmake_minimum_required(VERSION 2.8.3)
project(nubot_control)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

#set(CMAKE_BUILD_TYPE "Release")
#if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
#else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
#    set(CMAKE_BUILD_TYPE "Release")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
#endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  std_srvs
  nubot_common
  world_model
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
	         rospy  
		 std_msgs 
                 std_srvs
)

set(nubot_control_includes
               ${PROJECT_SOURCE_DIR}/include
               ${catkin_INCLUDE_DIRS}
               ${EIGEN_INCLUDE_DIRS}
)
set(nubot_control_src ${PROJECT_SOURCE_DIR}/src CACHE STRING "")        # To be used in other places

include_directories(${nubot_control_includes})

add_executable(nubot_control_node src/nubot_control.cpp
                                  src/role_assignment.cpp
                                  src/activerole.cpp
                                  src/fieldinformation.cpp
                                  src/strategy.cpp
                                  src/goaliestrategy.cpp
                                  src/behaviour.cpp
                                  src/plan.cpp
                                  src/subtargets.cpp
                                  src/bezier.cpp
                                  src/midfieldrole.cpp
                                  src/assistrole.cpp
                                  src/passiverole.cpp
                                  src/staticpass.cpp
)
add_dependencies(nubot_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nubot_control_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)


