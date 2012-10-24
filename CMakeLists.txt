cmake_minimum_required(VERSION 2.8.3)
project(warehouse_ros)

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  rostest
  rostime
  std_msgs)
find_package(Boost COMPONENTS system thread filesystem)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES warehouse_ros
  DEPENDS roscpp geometry_msgs rostime std_msgs Boost
  CFG_EXTRAS FindMongoDB.cmake)

include(cmake/FindMongoDB.cmake.in)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${MongoDB_INCLUDE_DIR})
link_directories(${catkin_LINK_DIRS})

set(warehouse_srcs
  src/mongo_ros.cpp)

add_library(warehouse_ros SHARED ${warehouse_srcs})

target_link_libraries(warehouse_ros ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${MongoDB_LIBRARIES})

catkin_add_gtest(test_mongo_roscpp test/test_mongo_ros.cpp)
target_link_libraries(test_mongo_roscpp warehouse_ros ${MongoDB_LIBRARIES})

# file(COPY src/wrapper.py DESTINATION ${CATKIN_BUILD_PREFIX}/lib/warehouse_ros)
# catkin_python_setup()
# install
# unit tests

install(TARGETS warehouse_ros LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS src/mongo_wrapper_ros.py DESTINATION bin)
