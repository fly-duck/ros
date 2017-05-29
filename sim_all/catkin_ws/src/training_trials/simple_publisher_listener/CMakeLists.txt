cmake_minimum_required(VERSION 2.8.3)
project(simple_publisher_listener)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})



add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(talker2 src/talker2.cpp)
target_link_libraries(talker2 ${catkin_LIBRARIES})
