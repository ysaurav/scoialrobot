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

#  start QT
find_package(Qt4 REQUIRED)
# enable/disable some Qt features
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

SET(QT_QRCS src/srgui/window_QT.qrc)
QT4_ADD_RESOURCES(SRC_QRCS ${QT_QRCS})

set(QT_SRCS src/srgui/gui.cpp src/srgui/imageframe.cpp src/srgui/window_QT.cpp src/SocialRobotGui.cpp ${SRC_QRCS})
set(QT_HDRS src/srgui/gui.h src/srgui/imageframe.h src/srgui/window_QT.h src/SocialRobotGui.h)

QT4_WRAP_CPP(QT_MOC_RSCS ${QT_HDRS})
QT4_WRAP_UI(QT_UIS src/srgui/gui.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/srgui)
# end QT

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_library(utils src/string_utils.cpp src/cv_utils.cpp src/kinect_proxy.cpp src/PixelSimilarity.cpp src/Template.cpp src/RosUtils.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(social_robot src/social_robot.cpp)
rosbuild_add_executable(social_robot_depth src/social_robot_depth.cpp)
rosbuild_add_executable(kinect_save_image src/kinect_save_image.cpp)
rosbuild_add_executable(social_robot_test src/social_robot_test.cpp)
# rosbuild_add_executable(calibration src/calib/calibration.cpp)
target_link_libraries(social_robot utils)
target_link_libraries(social_robot_depth utils)
target_link_libraries(kinect_save_image utils)
target_link_libraries(social_robot_test utils)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(qttest src/srgui/main.cpp ${QT_UIS} ${QT_SRCS} ${QT_MOC_RSCS})
target_link_libraries(qttest ${QT_LIBRARIES} utils)
