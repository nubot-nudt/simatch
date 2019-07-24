cmake_minimum_required(VERSION 2.8.3)
project(nubot_coach)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Debug")
#set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/bin")

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
#  coach_world_model
  world_model
  cmake_modules
)
 
find_package(Qt5Widgets)  
find_package(Qt5Network)
find_package(Qt5Xml)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
   INCLUDE_DIRS   include
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

FILE(GLOB_RECURSE QT_SOURCES   RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
FILE(GLOB_RECURSE QT_HEADERS   RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/nubot_coach/*.h)
FILE(GLOB         QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} source/*.qrc)
FILE(GLOB         QT_UI        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)

QT5_ADD_RESOURCES(COACH_RESOURCES ${QT_RESOURCES})
Qt5_WRAP_CPP     (COACH_HEADERS   ${QT_HEADERS})
Qt5_WRAP_UI      (COACH_UI        ${QT_UI})

add_executable(nubot_coach_node
     ${QT_SOURCES}
     ${COACH_RESOURCES}
     ${COACH_HEADERS}
     ${COACH_UI}
)

target_link_libraries(nubot_coach_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
qt5_use_modules(nubot_coach_node Widgets Network Xml)
 
