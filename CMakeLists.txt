cmake_minimum_required(VERSION 3.5)
project(jetson_camera_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


execute_process(
  COMMAND uname -m
  COMMAND tr -d '\n'
  OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Architecture: ${ARCHITECTURE}")

# find dependencies
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_components          REQUIRED)
find_package(ament_cmake                REQUIRED)
find_package(std_msgs                   REQUIRED)
find_package(sensor_msgs                REQUIRED)
find_package(camera_info_manager        REQUIRED)
find_package(image_transport            REQUIRED)
find_package(cv_bridge                  REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV 4                   REQUIRED)

message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
include_directories(include ${cv_bridge_INCLUDE_DIRS})

# Only build nodes on Jetson platform
if(${ARCHITECTURE} STREQUAL "aarch64")

  add_library(jetson_camera_driver SHARED 
    src/jetson_camera_driver.cpp)

  ament_target_dependencies(jetson_camera_driver
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_msgs"
    "camera_info_manager"
    "image_transport"
    "camera_calibration_parsers"
    "cv_bridge"
    "OpenCV")

  rclcpp_components_register_nodes(jetson_camera_driver "jetson_camera_driver::JetsonCameraNode")
  set(node_plugins "${node_plugins}jetson_camera_driver::JetsonCameraNode;$<TARGET_FILE:jetson_camera_driver>\n")

  add_executable(jetson_camera_driver_node
    src/jetson_camera_driver_node.cpp
  )

  # this need to be declared for generating executable
  target_link_libraries(jetson_camera_driver_node
    jetson_camera_driver
  )
  
  install(TARGETS 
    jetson_camera_driver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  install(TARGETS
    jetson_camera_driver_node
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # Install launch files.
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Install config files
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
