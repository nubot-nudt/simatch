cmake_minimum_required(VERSION 2.8.3)
project(coach4sim)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH} "/opt/Qt5.3.2/5.3/gcc_64/lib/cmake/Qt5Widgets")
#set(EXECUTABLE_OUTPUT_PATH  "${PROJECT_SOURCE_DIR}/bin")
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

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

if(Qt5Widgets_FOUND)
    message("${Green}I have found Qt5!${ColourReset}")
else()
    message("${Red}Cannot find QtWidgets. Please write its path to CMAKE_PREFIX_PATH here: ${PROJECT_SOURCE_DIR}/CMakeLists.txt.
    ${ColourReset}")
endif()

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
add_dependencies(nubot_coach_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nubot_coach_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
qt5_use_modules(nubot_coach_node Widgets Network Xml)
