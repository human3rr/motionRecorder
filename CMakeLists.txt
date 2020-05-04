cmake_minimum_required(VERSION 3.10)

project(motionRecorder)
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
add_executable(motionRecorder motionRecorder.cpp)
target_link_libraries( motionRecorder ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

