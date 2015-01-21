/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of hand_eye_calibration, a ROS package for hand eye calibration,

hand_eye_calibration is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
hand_eye_calibration is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with hand_eye_calibration. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "ros/ros.h"
#include "utils/numeric_iterator.h"
#include "utils/continuous_x_dim_space_iterator.h"
#include "hand_eye_calibration/dual_quaternion_transformation_estimator.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/CameraInfo.h>

/// class that autonomously extracts hand-eye pose correspondences in a robotic setup and estimates the hand-eye-calibration from it
class AutonomousHandEyeCalibrator
{
public:
  
  /// constructor
  /** Initializes the autonomous hand eye calibrator by reading all parameters from the parameter server
   * @param _n handle of the node the calibrator runs in
   * @throws ROS_FATAL if not all necessary parameters are given and shuts down the node
   */
  AutonomousHandEyeCalibrator( ros::NodeHandle* _n );
  
  /// runs one step of the autonomous calibration process
  /** The method iterates through the joint space and estimates the hand-eye transformation along the way.
   * Based on the hand-eye transformation it also estimates the position of the calibration target, e.g.
   * the checkerboard in order to skip joint positions where the target is expected not to be visible. The
   * current estimate is printed to the console. At each run another joint position is assumed.
   * @return false if all positions were covered
   */
  bool runSingleIteration();
  
  /// returns the maximal relative difference in any of the estimated transformation variables between previous and current estimate
  /** @return That is: max( (estimate_current-estimate_previous)/estimate_current )
   */
  double maxRelEstimateChange();
  
  /// writes all data to a file
  bool printToFile( std::string _filename );
  
private:
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> robot_;
  
  sensor_msgs::CameraInfo camera_info_; /// camera information from the camera pose publication node
  std::vector<geometry_msgs::Point> pattern_coordinates_; /// coordinates of the pattern used for calibration in its own 3d coordinate frame
  
  st_is::ContinuousXDimSpaceIterator< st_is::NumericIterator<double> > joint_position_; // current joint configuration of the robot
  DualQuaternionTransformationEstimator daniilidis_estimator_;
  std::vector< std::string > joint_names_;
  bool position_initialized_;
  
  /// loads the joint dimension names, limits and step sizes, initializes joint_position_ accordingly
  /** @throws ROS_FATAL if a correct configuration wasn't found on the parameter server and shuts down the node
   */
  void initializeJointSpace();
  
  /// loads joint specification from parameter server and initializes a new dimension in joint space
  /** @param name_ name of the dimension (=joint name)
   */
  void initializeJointConfiguration( std::string name_ );
  
  /// plans and executes a plan to the currently loaded target - blocks until completion
  /** completion means that the robot state is closer to the target than set in the tolerance and its velocity is approximately zero in all joint
   * @return true if movement was executed successfully
   */
  bool planAndMove();
  
  /// initializes the position to the current position of the arm
  /**
   * @throws ROS_ERROR If not all joints set for actuation are available through the MoveGroup
   */
  void initializePosition();
  
  /// calculates the next position and writes it into joint_position_
  /** (with no collisions, and where the calibration pattern is expected to be visible if calibration results
   * are already available
   * @return true if new position was found, false otherwhise (which means that the joint space was fully covered)
   */
  bool calculateNextJointPosition();
  
  /// returns whether MoveIt believes the robot state represented in _robot to be free of collisions or not given the current scene
  /**
   * @param _robot robot state to check
   * @param _scene the current scene
   */
  bool isCollisionFree( planning_scene::PlanningScenePtr _scene, robot_state::RobotState& _robot );
  
  /// calculates whether the calibration pattern is expected to be visible for the given robot state _robot
  /** @param _robot state of the robot 
   * @return true if expected to be visible or no knowledge of the calibration pattern position is available
   */
  bool calibrationPatternExpectedVisible( robot_state::RobotState& _robot );
    
  /// sets target position to the position encoded in position
  /** The new position to be assumed is the one in joint_position_
   */
  void setTargetToNewPosition( );
  
  
  /// sets the joint values in the robot state to the values currently pointed at by joint_positions_
  void setRobotStateToCurrentJointPosition( robot_state::RobotState& _robot );
  
  /// returns if camera pose publisher node info is available 
  /** If the information has not yet been gathered, it attempts to get information about the camera pose publisher node by calling the hand_eye_eye_node_info service
   */
  bool cameraPubNodeInfoAvailable();
  
  
};