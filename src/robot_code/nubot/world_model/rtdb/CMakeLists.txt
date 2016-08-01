add_subdirectory(parser)

add_library(rtdb STATIC rtdb_api.cpp)

add_library(multicast STATIC multicast.cpp)

add_executable(comm comm.cpp)
target_link_libraries(comm ${CMAKE_THREAD_LIBS_INIT} rtdb  multicast ${catkin_LIBRARIES} world_model)

add_executable(read read.cpp)
target_link_libraries(read rtdb  ${catkin_LIBRARIES} world_model)

add_executable(write write.cpp)
target_link_libraries(write rtdb  ${catkin_LIBRARIES} world_model)

