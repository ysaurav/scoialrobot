cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(utils src/string_utils.cpp src/cv_utils.cpp src/kinect_proxy.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(social_robot src/social_robot.cpp)
rosbuild_add_executable(kinect_save_image src/kinect_save_image.cpp)
rosbuild_add_executable(preprocessing src/preprocessing.cpp)
rosbuild_add_executable(chamfer src/chamfer.cpp)
rosbuild_add_executable(calibration src/calib/calibration.cpp)
target_link_libraries(social_robot utils)
target_link_libraries(kinect_save_image utils)
target_link_libraries(preprocessing utils)
target_link_libraries(chamfer utils)
#target_link_libraries(example ${PROJECT_NAME})
