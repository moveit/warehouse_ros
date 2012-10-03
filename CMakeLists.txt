cmake_minimum_required(VERSION 2.8)
project(warehouse)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  rostest
  rostime
  std_msgs)
find_package(Boost COMPONENTS system thread filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse
  DEPENDS rospy roscpp geometry_msgs rostime std_msgs Boost
  CFG_EXTRAS FindMongoDB.cmake)

include(cmake/FindMongoDB.cmake.in)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${MongoDB_INCLUDE_DIR})
link_directories(${catkin_LINK_DIRS})

set(warehouse_srcs
  src/mongo_ros.cpp)

add_library(warehouse SHARED ${warehouse_srcs})

target_link_libraries(warehouse ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${MongoDB_LIBRARIES})

catkin_add_gtest(test_mongo_roscpp test/test_mongo_ros.cpp)
target_link_libraries(test_mongo_roscpp warehouse ${MongoDB_LIBRARIES})

file(COPY src/wrapper.py DESTINATION ${CATKIN_BUILD_PREFIX}/lib/warehouse)
# catkin_python_setup()
# install
# unit tests

