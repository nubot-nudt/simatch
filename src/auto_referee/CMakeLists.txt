cmake_minimum_required(VERSION 2.8.3)
project(auto_referee)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_BUILD_TYPE "Debug")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nubot_common
  nubot_control
  gazebo_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(Curses REQUIRED)
find_package(OpenCV REQUIRED)
#link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS}
#                    ${GAZEBO_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
#                    ${CURSES_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    nubot_control
    gazebo_msgs
)				

add_executable(auto_referee src/auto_referee.cpp
                            src/Quaternion.cc
                            src/Helpers.cc
                            ${nubot_control_src}/fieldinformation.cpp

)
add_dependencies(auto_referee ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_referee ${catkin_LIBRARIES}
${OpenCV_LIBS}
									ncurses
                                   #${GAZEBO_LIBRARIES}
                                   #${CURSES_LIBRARIES}
)									
