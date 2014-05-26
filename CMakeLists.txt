cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(${PROJECT_SOURCE_DIR}/FindXercesC.cmake)

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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)

#include(${PROJECT_SOURCE_DIR}/FindXercesC.cmake)
#find_package(XercesC)
link_directories(/usr/lib)
include_directories(/usr/include/xercesc)



rosbuild_add_boost_directories()
#find_package(Protobuf REQUIRED)
rosbuild_add_executable(robot_embodiment 
						src/opencog_msgs.pai_msgs.pb.cc
						src/RobotEmbodiment.cpp
						src/OacPublisher.cpp
						src/OacSubscriber.cpp
						src/OacMessage.cpp
						src/Block.cpp
						src/Constants.cpp)
						
rosbuild_link_boost(robot_embodiment thread system)

set_target_properties(robot_embodiment -lxerces-c)
target_link_libraries(robot_embodiment protobuf libxerces28 libxerces-c)

#libxerces-c
#rosbuild_add_executable(people_detector src/people_detector.cpp)
#target_link_libraries(example ${PROJECT_NAME})

