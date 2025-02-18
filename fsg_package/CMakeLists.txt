cmake_minimum_required(VERSION 3.5)
project(fsg_package)

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

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(test_input src/test_input.cpp)
ament_target_dependencies(test_input rclcpp std_msgs image_transport cv_bridge "OpenCV")
target_link_libraries(test_input ${OpenCV_LIBRARIES})

target_include_directories(test_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
  $<INSTALL_INTERFACE:include>)

file(GLOB_RECURSE LIB_SOURCES "include/*.cpp" "include/*.h")
include_directories(include)
add_library(fsg_lib ${LIB_SOURCES})
target_link_libraries(fsg_lib ${OpenCV_LIBS})

add_executable(fsg src/fsg.cpp)
ament_target_dependencies(fsg rclcpp std_msgs image_transport cv_bridge "OpenCV")
target_link_libraries(fsg fsg_lib ${OpenCV_LIBRARIES})

target_include_directories(fsg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}
  $<INSTALL_INTERFACE:include>)

install(TARGETS fsg test_input
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
