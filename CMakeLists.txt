cmake_minimum_required(VERSION 2.8.3)
project(warehouse_ros)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  rostest
  rostime
  std_msgs)
find_package(Boost COMPONENTS system filesystem thread)
find_package(OpenSSL)
find_package(MongoDB)

set(MONGO_EXPORT)
if("${MongoDB_LIBRARIES}" MATCHES "\\.so$")
  set(MONGO_EXPORT MongoDB)
endif()

file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include include
  LIBRARIES warehouse_ros
  CATKIN_DEPENDS roscpp geometry_msgs rostime std_msgs
  DEPENDS Boost ${MONGO_EXPORT}
  )


if (NOT MongoDB_EXPOSE_MACROS)
  add_definitions(-DMONGO_EXPOSE_MACROS)
endif()
configure_file("include/mongo_ros/config.h.in" "${CATKIN_DEVEL_PREFIX}/include/mongo_ros/config.h")

include_directories(${CATKIN_DEVEL_PREFIX}/include include ${catkin_INCLUDE_DIRS} ${MongoDB_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(warehouse_srcs
  src/mongo_ros.cpp
  src/mongo_ros_dummy.cpp)

add_library(warehouse_ros SHARED ${warehouse_srcs})

target_link_libraries(warehouse_ros ${catkin_LIBRARIES} ${MongoDB_LIBRARIES} ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES})
catkin_add_gtest(test_mongo_roscpp test/test_mongo_ros.cpp)
target_link_libraries(test_mongo_roscpp warehouse_ros)

install(TARGETS warehouse_ros LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(PROGRAMS src/mongo_wrapper_ros.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES "${CATKIN_DEVEL_PREFIX}/include/mongo_ros/config.h" DESTINATION include/mongo_ros)
