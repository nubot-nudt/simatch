cmake_minimum_required(VERSION 2.8.3)
project(nubot_hwcontroller)
#  std=gnu++0x" enables c++ 11
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
  dynamic_reconfigure
  realtime_tools
  nubot_common
)

add_message_files(FILES
   DebugInfo.msg
 )

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  cfg/controller.cfg
)

catkin_package(
 INCLUDE_DIRS   include
 CATKIN_DEPENDS roscpp
                std_msgs
                message_generation
 DEPENDS        realtime_tools
                nubot_common
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)



add_executable(nubot_hwcontroller_node src/nubot_hwcontroller_node.cpp
                                      )


add_dependencies(nubot_hwcontroller_node ${PROJECT_NAME}_gencfg)
add_dependencies(nubot_hwcontroller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(nubot_hwcontroller_node
   ${catkin_LIBRARIES}
   ncurses
)
