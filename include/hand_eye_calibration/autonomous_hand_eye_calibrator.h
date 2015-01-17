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
  
  st_is::ContinuousXDimSpaceIterator< st_is::NumericIterator<double> > joint_position_; // current joint configuration of the robot
  DualQuaternionTransformationEstimator daniilidis_estimator_;
  std::vector< std::string > joint_names_;
  
  /// loads the joint dimension names, limits and step sizes, initializes joint_position_ accordingly
  /** @throws ROS_FATAL if a correct configuration wasn't found on the parameter server and shuts down the node
   */
  void initializeJointSpace();
  
  /// loads joint specification from parameter server and initializes a new dimension in joint space
  /** @param name_ name of the dimension (=joint name)
   */
  void initializeJointConfiguration( std::string name_ );
  
};