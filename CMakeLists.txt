cmake_minimum_required(VERSION 3.10.2)
project(warehouse_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(OpenSSL REQUIRED)

include_directories(
  include
  ${OPENSSL_INCLUDE_DIR}
)

set(warehouse_srcs
  src/database_loader.cpp
  src/transform_collection.cpp)

add_library(warehouse_ros SHARED ${warehouse_srcs})
ament_target_dependencies(warehouse_ros
  rclcpp
  std_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  Boost
)
target_link_libraries(warehouse_ros ${OPENSSL_CRYPTO_LIBRARY})

add_executable(warehouse_test_dbloader src/test_dbloader.cpp)
ament_target_dependencies(warehouse_test_dbloader
  rclcpp
  std_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  Boost
)
target_link_libraries(warehouse_test_dbloader warehouse_ros ${OPENSSL_CRYPTO_LIBRARY})

ament_export_include_directories(include)
ament_export_libraries(warehouse_ros)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(Boost)
ament_export_dependencies(OpenSSL)

install(
  TARGETS warehouse_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS warehouse_test_dbloader
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
