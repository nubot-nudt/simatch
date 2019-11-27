cmake_minimum_required(VERSION 2.8.3)
project(nubot_gazebo)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
  nubot_common
  dynamic_reconfigure
)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(config/nubot_gazebo.cfg)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

message("gazebo include path: ${GAZEBO_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
    roscpp
    dynamic_reconfigure
  DEPENDS 
    gazebo
)				

add_library(nubot_gazebo src/nubot_gazebo.cc)
add_dependencies(nubot_gazebo ${PROJECT_NAME}_gencfg)
add_dependencies(nubot_gazebo  ${catkin_EXPORTED_TARGETS})
target_link_libraries(nubot_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

add_library(ball_gazebo src/ball_gazebo.cc)
add_dependencies(ball_gazebo ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(nubot_teleop_keyboard src/nubot_teleop_keyboard.cc)
add_dependencies(nubot_teleop_keyboard  ${catkin_EXPORTED_TARGETS})
target_link_libraries(nubot_teleop_keyboard ${catkin_LIBRARIES})


# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#	pkg_check_modules(GAZEBO gazebo)
# endif()
