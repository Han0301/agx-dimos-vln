cmake_minimum_required(VERSION 3.8)
project(compressed_RGBD_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(compressed_rgbd_node src/compressed_rgbd_node.cpp)
target_include_directories(compressed_rgbd_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(
  compressed_rgbd_node
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${OpenCV_LIBS}
)

install(TARGETS compressed_rgbd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
