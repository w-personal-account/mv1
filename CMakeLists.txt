cmake_minimum_required(VERSION 3.0.2)
project(lsc_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  diagnostic_updater
  self_test
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}/include/}
)

add_executable(lsc_laser_publisher
        src/lsc_laser_publisher.cpp
        src/laser.cpp
        src/parser.cpp
        src/socket.cpp
        )

add_dependencies(lsc_laser_publisher
          ${${PROJECT_NAME}_EXPORTED_TARGETS}
          ${catkin_EXPORTED_TARGETS}
          )

target_link_libraries(lsc_laser_publisher
              ${catkin_LIBRARIES}
              )
