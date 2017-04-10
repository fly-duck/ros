cmake_minimum_required(VERSION 2.4.6)
project(pf2dlocalizer)

find_package(cmake_modules REQUIRED)

find_package(PkgConfig REQUIRED)
# pkg-config support
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs 
  nav_msgs 
  visualization_msgs 
  sensor_msgs
  random_numbers
  tf
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Coverage)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#generate_messages(DEPENDENCIES 
#  std_msgs 
#  sensor_msgs
#)

catkin_package(CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  rospy
  std_msgs 
  nav_msgs 
  visualization_msgs
  sensor_msgs
  tf
) 
include_directories(${catkin_INCLUDE_DIRS})

## System dependencies found with CMake's conventions
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

SET ( CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-deprecated -g" )
	
add_executable(pf2dlocalizerNode src/main.cpp src/pf2dlocalizer.cpp src/pf.cpp src/wpose.cpp)
target_link_libraries(pf2dlocalizerNode ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${OpenCV_LIBRARIES} ${yaml-cpp_LIBRARIES})
#add_dependencies(pf2dlocalizerNode ${PROJECT_NAME}_generate_messages_cpp)

