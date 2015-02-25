cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)

list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  geometry_msgs
  roscpp
  tf2_msgs
  tf2_ros
  moveit_ros_planning_interface
  control_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system ) #accumulators bind ref)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  CameraPoseDescription.msg
  CameraPoseNodeInfo.msg
  NumberedRequest.msg
  Point2D.msg
  HandPoseDescription.msg
)

add_service_files(
  FILES
  CameraPose.srv
  CameraPoseInfo.srv
  HandPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf2_msgs tf2_ros sensor_msgs cv_bridge moveit_ros_planning_interface control_msgs message_runtime
  DEPENDS opencv opencv2 Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(youbot_link_position_publisher
  src/hand_eye_calibration/code_base/youbot_link_position_publisher.cpp
  src/hand_eye_calibration/ros_nodes/youbot_link_position_publisher_node.cpp
  src/utils/ros_eigen.cpp
  )

target_link_libraries(youbot_link_position_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(youbot_link_position_publisher
 ${catkin_EXPORTED_TARGETS}
)

add_executable(tf2_republisher
  src/hand_eye_calibration/code_base/tf2_republisher.cpp
  src/hand_eye_calibration/ros_nodes/tf2_republisher_node.cpp
)

target_link_libraries(tf2_republisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(tf2_republisher
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(eye_position_from_checkerboard
  src/hand_eye_calibration/code_base/eye_position_from_checkerboard.cpp
  src/hand_eye_calibration/ros_nodes/eye_position_from_checkerboard_node.cpp
  src/utils/ros_eigen.cpp
)

target_link_libraries(eye_position_from_checkerboard
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(eye_position_from_checkerboard
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(dual_quaternion_transformation_estimator
  src/hand_eye_calibration/code_base/transformation_estimator.cpp
  src/hand_eye_calibration/code_base/estimation_data.cpp
  src/hand_eye_calibration/code_base/calibration_setup.cpp
  src/hand_eye_calibration/code_base/dual_quaternion_transformation_estimator.cpp
  src/hand_eye_calibration/ros_nodes/dual_quaternion_estimation_node.cpp
  src/hand_eye_calibration/code_base/pose_creator.cpp
  src/utils/eigen_utils.cpp
  src/utils/math.cpp
  src/utils/ros_eigen.cpp
)

target_link_libraries(dual_quaternion_transformation_estimator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(dual_quaternion_transformation_estimator
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(autonomous_hand_eye_calibration
  src/hand_eye_calibration/ros_nodes/autonomous_calibration.cpp
  src/hand_eye_calibration/code_base/autonomous_hand_eye_calibrator.cpp
  src/hand_eye_calibration/code_base/dual_quaternion_transformation_estimator.cpp
  src/hand_eye_calibration/code_base/transformation_estimator.cpp
  src/hand_eye_calibration/code_base/estimation_data.cpp
  src/hand_eye_calibration/code_base/calibration_setup.cpp
  src/hand_eye_calibration/code_base/pose_creator.cpp
  src/utils/eigen_utils.cpp
  src/utils/math.cpp
  src/utils/ros_eigen.cpp
)

target_link_libraries(autonomous_hand_eye_calibration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(autonomous_hand_eye_calibration
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(transformation_test
  src/hand_eye_calibration/ros_nodes/transformation_test.cpp
  src/hand_eye_calibration/code_base/autonomous_hand_eye_calibrator.cpp
  src/hand_eye_calibration/code_base/dual_quaternion_transformation_estimator.cpp
  src/hand_eye_calibration/code_base/transformation_estimator.cpp
  src/hand_eye_calibration/code_base/estimation_data.cpp
  src/hand_eye_calibration/code_base/calibration_setup.cpp
  src/hand_eye_calibration/code_base/pose_creator.cpp
  src/utils/eigen_utils.cpp
  src/utils/math.cpp
  src/utils/ros_eigen.cpp
)

target_link_libraries(transformation_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(transformation_test
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(fake_pose_server
  src/hand_eye_calibration/ros_nodes/fake_pose_server.cpp
  src/hand_eye_calibration/code_base/transformation_estimator.cpp
  src/hand_eye_calibration/code_base/estimation_data.cpp
  src/hand_eye_calibration/code_base/calibration_setup.cpp
  src/hand_eye_calibration/code_base/pose_creator.cpp
  src/utils/eigen_utils.cpp
  src/utils/math.cpp
  src/utils/ros_eigen.cpp
)

target_link_libraries(fake_pose_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(fake_pose_server
 ${catkin_EXPORTED_TARGETS}
 ${PROJECT_NAME}_generate_messages_cpp
)