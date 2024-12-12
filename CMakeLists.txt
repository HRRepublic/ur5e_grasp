cmake_minimum_required(VERSION 3.0.2)
project(my_ur5_moveit_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
) 

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_grasp_node src/simple_grasp.cpp)
target_link_libraries(simple_grasp_node
  ${catkin_LIBRARIES}
)

add_executable(test_simple_grasp_node src/test_simple_grasp.cpp)
target_link_libraries(test_simple_grasp_node
  ${catkin_LIBRARIES}
)

