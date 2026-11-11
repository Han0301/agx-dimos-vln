cmake_minimum_required(VERSION 3.5)
project(insta360_ros_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
option(BUILD_INSTA360_CAMERA "Build X3 SDK/FFmpeg camera executables" ON)
if(BUILD_INSTA360_CAMERA)
  pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
  pkg_check_modules(AVFORMAT REQUIRED IMPORTED_TARGET libavformat)
  pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)
  pkg_check_modules(SWSCALE REQUIRED IMPORTED_TARGET libswscale)
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
)

install(
  DIRECTORY config launch lib
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_INSTA360_CAMERA)
  add_executable(${PROJECT_NAME} src/main.cpp)
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    cv_bridge
    sensor_msgs
    std_msgs
    OpenCV
  )
  target_link_libraries(${PROJECT_NAME} CameraSDK)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../share/${PROJECT_NAME}/lib"
  )

  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(decoder src/decoder.cpp)
  ament_target_dependencies(decoder
    rclcpp
    cv_bridge
    sensor_msgs
    std_msgs
    OpenCV
  )
  target_link_libraries(decoder ${OpenCV_LIBRARIES} PkgConfig::AVCODEC PkgConfig::AVFORMAT PkgConfig::AVUTIL PkgConfig::SWSCALE)

  install(TARGETS decoder
    DESTINATION lib/${PROJECT_NAME}
  )

  # Add C++ equirectangular node
  add_executable(equirectangular_cpp src/equirectangular.cpp)
  ament_target_dependencies(equirectangular_cpp
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
  )
  target_link_libraries(equirectangular_cpp ${OpenCV_LIBRARIES})

  install(TARGETS equirectangular_cpp
    DESTINATION lib/${PROJECT_NAME}
  )

  # Publishes four rectilinear 90-degree views from the equirectangular image
  add_executable(surround_views src/surround_views.cpp)
  ament_target_dependencies(surround_views
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
  )
  target_link_libraries(surround_views ${OpenCV_LIBRARIES})

  install(TARGETS surround_views
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

# D455 launch files also use these Python diagnostics and readiness helpers.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/equirectangular.py
  scripts/calibrate.py
  scripts/calibrate_cpp.py
  scripts/wait_for_camera_data.py
  scripts/camera_dashboard.py
  scripts/dual_fisheye_combiner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
