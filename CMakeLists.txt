cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

set(SDK ARDroneLib/)
link_directories(${PROJECT_SOURCE_DIR}/lib/)
include_directories(${SDK} ${SDK}/FFMPEG/Includes ${SDK}/Soft/Common ${SDK}/Soft/Lib ${SDK}/VP_SDK ${SDK}/VP_SDK/VP_Os/linux )
rosbuild_add_executable(ardrone_driver src/ardrone_driver.cpp src/video.cpp src/ardrone_sdk.cpp src/teleop_twist.cpp)
rosbuild_add_executable(ardrone_teleop_key src/ardrone_teleop_key.cpp)
rosbuild_add_executable(ardrone_opencv src/ardrone_opencv.cpp)
rosbuild_add_executable(ardrone_tracker src/ardrone_tracker.cpp)
rosbuild_add_executable(ardrone_follow src/ardrone_follow.cpp)
rosbuild_add_executable(ardrone_tracker_sim src/ardrone_tracker_sim.cpp)
rosbuild_add_executable(ardrone_tracker_control src/ardrone_tracker_control.cpp)

find_package(OpenCV)

#target_link_libraries(ardrone_driver vlib pc_ardrone sdk SDL avcodec avdevice avfilter avformat avutil swscale)
target_link_libraries(ardrone_driver pc_ardrone avcodec avutil swscale vlib  sdk SDL )
target_link_libraries(ardrone_opencv ${OpenCV_LIBRARIES})
target_link_libraries(ardrone_tracker ${OpenCV_LIBRARIES})
target_link_libraries(ardrone_tracker_control ${OpenCV_LIBRARIES})



