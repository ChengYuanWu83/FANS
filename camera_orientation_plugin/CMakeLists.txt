cmake_minimum_required(VERSION 3.0.2)
project(camera_orientation_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/CameraOrientationPlugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
