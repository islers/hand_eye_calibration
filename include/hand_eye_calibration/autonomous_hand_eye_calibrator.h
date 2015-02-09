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
#include <Eigen/Core>

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
  
  /** adds a transformation estimator
   * @param _new_estimator the estimator
   */
  void addTransformationEstimator( boost::shared_ptr<TransformationEstimator> _new_estimator );
  
  /** adds a transformation estimator using the method given
   * @param _method transformation method
   */
  void addTransformationEstimationMethod( boost::shared_ptr<TransformationEstimator::EstimationMethod> _method );
  
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
  
  /** Sets a new image border tolerance. The tolerance is used when deciding whether projected image coordinates of calibration pattern points are expected to be visible in the image or not. Default is 50.
   * @param _tolerance the border tolerance
   */
  void setImageBorderTolerance( double _tolerance );
  
  /** Returns the current estimate
   */
  TransformationEstimator::EstimationData getEstimate();
  
  /** Returns whether an estimate is available or not
   */
  bool estimateAvailable();
  
  /** returns the number of pose pairs added so far
   */
  unsigned int count();
  
  /// writes all data to a file
  bool printToFile( std::string _filename );
  
private:
  ros::NodeHandle* ros_node_;
  ros::ServiceClient eye_client_;
  ros::ServiceClient hand_client_;
  
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> scene_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> robot_;
    
  st_is::ContinuousXDimSpaceIterator< st_is::NumericIterator<double> > joint_position_; // current joint configuration of the robot
  std::list< boost::shared_ptr<TransformationEstimator> > estimators_; // container for all estimators that are to be used, the first one is used as source for calculations
  std::vector< std::string > joint_names_;
  bool position_initialized_; /// true if the initializePosition()-method was successfully run
  
  double image_border_tolerance_; /// by what value [px] the border of the image shall be enlarged when deciding whether a projected calibration pattern point might be part of the image or not - loaded from param server if available (param:'image_border_tolerance'), default is 40 px
  double pattern_safety_distance_square_; /// minimal distance the camera may have from calculated pattern positions [m] - the unsquared version is loaded from param server if available (param:'pattern_safety_distance'), default is 0.3 m
  
  std::string robot_base_frame_; /// name of the robot base frame used by the hand pose publisher
  std::string robot_hand_frame_; /// name of the robot hand frame used by the hand pose publisher
  
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
  
  /// calculates the next valid position and writes it into joint_position_
  /** (with no collisions, where the calibration pattern is expected to be visible and the camera position is not too close or behind the pattern if calibration results are already available)
   * @return true if new position was found, false otherwhise (which means that the joint space was fully covered)
   */
  bool calculateNextJointPosition();
  
  /// calculates the calibration pattern coordinates in camera frame coordinates given a robot state
  /**
   * @param _robot robot state
   * @return camera pose:   The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO,  Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   */
  geometry_msgs::Pose getCameraWorldPose( robot_state::RobotState& _robot );
  
  /// calculates the calibration pattern point coordinates in camera frame given a robot configuration
  /**
   * @param _robot robot state
   * @param _pattern_coordinates Vector that gets filled with the coordinates (using push_back() ) - the vector is not emptied: No elements are added if no camera calibration setting info is available or an estimation isn't possible yet
   */
  void getCameraFrameCoordinates( robot_state::RobotState& _robot, std::vector<geometry_msgs::Point>& _pattern_coordinates );
  
  /// returns whether MoveIt believes the robot state represented in _robot to be free of collisions or not given the current scene (but without the calibration pattern)
  /**
   * @param _robot robot state to check
   * @param _scene the current scene
   */
  bool isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot );
  
  
  /// returns whether the origin of the camera frame is sufficiently far away from all given pattern coordinates and not in front of any (no points witz z<0)
  /**
   * @param _pattern_coordinates the coordinates of the pattern
   * @return true if the relative position seems ok
   */
  bool relativePositionToPatternAcceptable( std::vector<geometry_msgs::Point>& _pattern_coordinates );
  
  /// calculates whether the calibration pattern is expected to be visible for the given robot state _robot
  /** @param _pattern_coordinates coordinates of the calibration pattern in camera frame coordinates
   * @return true if expected to be visible or no knowledge of the calibration pattern position is available
   */
  bool calibrationPatternExpectedVisible( std::vector<geometry_msgs::Point>& _pattern_coordinates );
    
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