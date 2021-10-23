cmake_minimum_required(VERSION 3.0.2)
project(aruco_test)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  message_generation
)
add_message_files(
  FILES
  position.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

find_package(OpenCV  REQUIRED)
find_package(aruco  REQUIRED )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_INCLUDE_DIRS}
)

add_executable(aruco_test src/aruco_test.cpp)
target_link_libraries(aruco_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${aruco_LIBS}
)
add_dependencies(aruco_test ${PROJECT_NAME}_gencpp)

add_executable(aruco_dev src/aruco_dev.cpp)
target_link_libraries(aruco_dev
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${aruco_LIBS}
)
add_dependencies(aruco_dev ${PROJECT_NAME}_gencpp)