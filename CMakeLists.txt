cmake_minimum_required(VERSION 3.0.2)
project(hand_eye_grasp)


 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  geometric_shapes
  image_transport
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  object_recognition_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  aruco_ros
  pcl_conversions
  pcl_ros
  pcl_msgs
  tf2   
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)


FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)


catkin_package( INCLUDE_DIRS include   LIBRARIES hand_eye_grasp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include  ${catkin_INCLUDE_DIRS} )

#include_directories(include)
include_directories(  SYSTEM  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
include_directories(${kinova_driver_INCLUDE_DIRS})
include_directories(include ${PCL_INCLUDE_DIRS})
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(find_four_aruco_marker_four src/find_four_aruco_marker_four.cpp)
target_link_libraries(find_four_aruco_marker_four ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(translate_markers_to_target_frame src/translate_markers_to_target_frame.cpp)
target_link_libraries(translate_markers_to_target_frame ${catkin_LIBRARIES})
add_dependencies(translate_markers_to_target_frame ${catkin_EXPORTED_TARGETS})

add_executable(filter_cloud src/filter_cloud.cpp)
target_link_libraries(filter_cloud ${catkin_LIBRARIES})
add_dependencies(filter_cloud ${catkin_EXPORTED_TARGETS})

add_executable(marker_possible_place src/marker_possible_place.cpp)
target_link_libraries(marker_possible_place yaml-cpp  ${catkin_LIBRARIES})

add_library(pick_place  src/pick_place.cpp  )


add_executable(pick_place_on_markers_demo src/pick_place_on_markers_demo.cpp include/pick_place.h src/pick_place.cpp )
target_link_libraries(pick_place_on_markers_demo ${catkin_LIBRARIES})
add_dependencies(pick_place_on_markers_demo  ${${hand_eye_grasp}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 


add_executable(find_object src/find_object.cpp include/find_object.h)
target_link_libraries(find_object ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${kinova_driver_LIBRARIES})

