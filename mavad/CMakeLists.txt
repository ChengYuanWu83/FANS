project(mavad)
cmake_minimum_required(VERSION 3.18)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  planner_msgs
  image_transport
  cv_bridge
)

find_package(yaml-cpp REQUIRED)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  c_library_v2
)

file(GLOB_RECURSE mavades  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

set(DONT_BUILD )
set(libraries_to_link ${libwifi} ${libapplications} ${libolsr} ${libnetanim} ${libinternet} ${libflow-monitor} ${libcore} ${libmobility} ${libconfig-store} ${libstats} ${libcsma} ${libbridge})

add_library(planner_ns3_utils SHARED src/planner_ns3_utils.cc)
add_library(planner_config    SHARED src/planner_config.cc)
add_library(planner_ns3       SHARED src/planner_ns3.cc)
add_library(drone_planner     SHARED src/drone_planner.cc)
add_library(gcs_planner       SHARED src/gcs_planner.cc)

target_link_libraries(planner_ns3_utils ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(planner_config    ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(drone_planner     ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(gcs_planner       ${ns3-libs}         ${ns3-contrib-libs})
target_link_libraries(planner_ns3       ${catkin_LIBRARIES} ${ns3-libs} "${libraries_to_link}" ${ns3-contrib-libs}   planner_config planner_ns3_utils yaml-cpp drone_planner gcs_planner)

add_executable(mavad_main src/mavad_main.cc)
target_link_libraries(mavad_main ${catkin_LIBRARIES} planner_ns3_utils planner_config planner_ns3 drone_planner gcs_planner)

