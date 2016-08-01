cmake_minimum_required(VERSION 2.8.3)
project(nubot_coach)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} "/opt/Qt5.3.2/5.3/gcc_64/lib/cmake/Qt5Widgets/")
#set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/bin")
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_filters
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  nubot_common
  world_model
  cmake_modules
)
 
find_package(Qt5Widgets)  
find_package(Qt5Network)
find_package(Qt5Xml)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_generation
                  dynamic_reconfigure
                  roscpp
                  rospy
                  std_msgs
                  std_srvs
)


set(nubot_coach_includes
               ${catkin_INCLUDE_DIRS}
               ${Boost_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include/nubot_coach
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${nubot_coach_includes})


FILE(GLOB_RECURSE CPP_SRC_LIST "src/*.cpp")

Qt5_WRAP_CPP(coach_MOCS include/nubot_coach/coach_dialog.h)
Qt5_WRAP_UI(coach_UIS src/coach_dialog.ui)
add_executable(nubot_coach_node
     ${CPP_SRC_LIST}
     ${coach_UIS}
     ${coach_MOCS}
)

target_link_libraries(nubot_coach_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
qt5_use_modules(nubot_coach_node Widgets Network Xml)
 
