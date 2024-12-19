cmake_minimum_required(VERSION 3.0.2)
project(my_ur5_moveit_demo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_ros_planning_interface
  roscpp
  cv_bridge
  image_transport
  tf
) 

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(simple_grasp_node src/simple_grasp.cpp)
target_link_libraries(simple_grasp_node
  ${catkin_LIBRARIES}
)

add_executable(oakd_object_detector_node src/oakd_object_detector.cpp)
target_link_libraries(oakd_object_detector_node
  ${catkin_LIBRARIES}
)

add_executable(test_simple_grasp_node src/test_simple_grasp.cpp)
target_link_libraries(test_simple_grasp_node
  ${catkin_LIBRARIES}
)

add_executable(calibration_data_recorder_node src/calibration_data_recorder.cpp)
target_link_libraries(calibration_data_recorder_node
  ${catkin_LIBRARIES}
)

add_executable(calibration_data_recorder_chess_node src/calibration_data_recorder_chess.cpp)
target_link_libraries(calibration_data_recorder_chess_node
  ${catkin_LIBRARIES}
)

add_executable(test_calib_result_node src/test_calib_result.cpp)
target_link_libraries(test_calib_result_node
  ${catkin_LIBRARIES}
)
