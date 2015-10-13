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
rosbuild_add_library(cppSeaSnakeComm include/UdpConnection.cpp
                        include/DelimitedMessageConnection.cpp
                        include/google/protobuf/message_lite.cc
                        include/google/protobuf/generated_message_util.cc
                        include/google/protobuf/wire_format_lite.cc
                        include/google/protobuf/io/coded_stream.cc
                        include/google/protobuf/io/zero_copy_stream.cc
                        include/google/protobuf/io/zero_copy_stream_impl.cc
                        include/google/protobuf/io/zero_copy_stream_impl_lite.cc
                        include/google/protobuf/stubs/common.cc
                        include/google/protobuf/stubs/once.cc
                        include/google/protobuf/stubs/atomicops_internals_x86_gcc.cc
			include/google/protobuf/repeated_field.cc
                        include/protocol/Bootloader.pb.cpp
                        include/protocol/Core.pb.cpp
                        include/protocol/Control.pb.cpp                     
                        include/protocol/DataTypes.pb.cpp
                        include/protocol/Settings.pb.cpp
                        include/protocol/RemoteServices.pb.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(mmc_comm_node src/mmc_comm_main.cpp)
target_link_libraries(mmc_comm_node cppSeaSnakeComm)

#rosbuild_add_executable(mmc_comm_node_beta src/mmc_comm_main_beta.cpp)
#target_link_libraries(mmc_comm_node_beta cppSeaSnakeComm)

#target_link_libraries(example ${PROJECT_NAME})
