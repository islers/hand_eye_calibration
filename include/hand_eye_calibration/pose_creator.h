/* Copyright (c) 2014, 2015, Stefan Isler, islerstefan@bluewin.ch
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

#include <Eigen/Core>
#include "utils/eigen_utils.h"
#include "geometry_msgs/Pose.h"
#include "hand_eye_calibration/transformation_estimator.h"

/// class to create artificial hand-eye pose pair sets
class PoseCreator
{
public:
  /** empty constructor, initializes world-robot and hand-eye poses randomly, using mersenne twister with random seed
   */
  PoseCreator();
  
  /** set the transformations when initializing, mersenne twister seed set to random
   * @param _t_WB transformation from robot base to world coordinates (world=pattern space)
   * @param _t_HE transformation from eye to hand coordinates
   */
  PoseCreator( st_is::CoordinateTransformation _t_WB, st_is::CoordinateTransformation _t_HE );
  
  /** set the transformations when initializing, mersenne twister seed set to random
   * @param _t_WB transformation from robot base to world coordinates (world=pattern space)
   * @param _t_HE transformation from eye to hand coordinates
   */
  PoseCreator( geometry_msgs::Pose _t_WB, geometry_msgs::Pose _t_HE );
  
  /** set the transformations when initializing, mersenne twister seed set to random
   * @param _t_WB transformation from robot base to world coordinates (world=pattern space)
   * @param _t_HE transformation from eye to hand coordinates
   */
  PoseCreator( Eigen::Matrix<double,3,4> _t_WB, Eigen::Matrix<double,3,4> _t_HE );
  
  /** sets the world base transformation
   * @param _t_WB transformation from robot base to world coordinates (world=pattern space)
   */
  void setWorldBaseTransform( geometry_msgs::Pose _t_WB );
  
  /** sets the world base transformation
   * @param _t_WB transformation from robot base to world coordinates (world=pattern space)
   */
  void setWorldBaseTransform( Eigen::Matrix<double,3,4> _t_WB );
  
  /** sets the hand eye transformation
   * @param _t_HE transformation from eye to hand coordinates
   */
  void setHandEyeTransform( geometry_msgs::Pose _t_HE );
  
  /** sets the hand eye transformation
   * @param _t_HE transformation from eye to hand coordinates
   */
  void setHandEyeTransform( Eigen::Matrix<double,3,4> _t_HE );
  
  /** returns the currently set transformation from robot base to world
   */
  void getWorldBaseTransform( geometry_msgs::Pose& _t_WB );
  
  /** returns the currently set transformation from robot base to world
   */
  void getWorldBaseTransform( Eigen::Matrix<double,3,4>& _t_WB );
  
  /** returns the currently set transformation from robot base to world
   */
  void getWorldBaseTransform( st_is::CoordinateTransformation& _t_WB );
  
  /** returns the currently set transformation from eye to hand
   */
  void getHandEyeTransform( geometry_msgs::Pose& _t_HE );
  
  /** returns the currently set transformation from eye to hand
   */
  void getHandEyeTransform( Eigen::Matrix<double,3,4>& _t_HE );
  
  /** returns the currently set transformation from eye to hand
   */
  void getHandEyeTransform( st_is::CoordinateTransformation& _t_HE );
  
  /** returns a reference to the calculated pose pairs, doesn't calculate anything
   */
  std::vector<TransformationEstimator::PoseData>& posePairs();
  
  /** calculates new pose pairs, the old one are deleted
   * @param _number_of_pairs how many pairs to calculate
   * @param _rotation_bounds uniform distribution borders for the rotations in pi
   * @param _translation_bounds uniform distribution borders for the translations
   */
  std::vector<TransformationEstimator::PoseData>& calcPosePairs( int _number_of_pairs, std::pair<double,double> _rotation_bounds = std::pair<double,double>(-2,2), std::pair<double,double> _translation_bounds = std::pair<double,double>(0,4) );
  
  /** adds normally distributed additive noise to the generated hand and eye poses
   * @param _rotation_noise standard deviation of the noise applied to the orientation in all axis
   * @param _translation_noise standard deviation of the noise added to the translation in all dimensions
   */
  void addNoise( double _rotation_noise=0, double _translation_noise=0 );
  
  /** adds normally distributed additive noise to the generated hand poses
   * @param _rotation_noise standard deviation of the noise applied to the orientation in all axis
   * @param _translation_noise standard deviation of the noise added to the translation in all dimensions
   */
  void addHandNoise( double _rotation_noise=0, double _translation_noise=0 );
  
  /** adds normally distributed additive noise to the generated eye poses
   * @param _rotation_noise standard deviation of the noise applied to the orientation in all axis
   * @param _translation_noise standard deviation of the noise added to the translation in all dimensions
   */
  void addEyeNoise( double _rotation_noise=0, double _translation_noise=0 );
  
  /** sets a fixed seed value for the internal mersenne twister random number generator
   * @param _seed_value seed value
   */
  void setSeed( int _seed_value );
  
  /** sets the internal seed back to random (default)
   */
  void setRandomSeed();
  
  /** saves the pose pairs to two files named _filename_hand.txt and _filename_eye.txt
   */
  void toFile( std::string _filename );
  
  /** reads pose pairs from the two files named _filename_hand.txt and _filename_eye.txt, Old data will be deleted if reading doesn't yield the same number of eye and hand poses
   * @throws std::runtime_error if the extracted data vectors are not of the same size
   */
  void fromFile( std::string _filename );
  
  /* sets the internal pose pairs
   */
  void setPosePairs( std::vector<TransformationEstimator::PoseData>& _pose_data );
  
private:
  st_is::CoordinateTransformation t_WB_; // world (pattern) - robot base
  st_is::CoordinateTransformation t_HE_; // hand-eye
  
  double random_seed_;
  int seed_value_;
  
  double hand_rot_stddev_; /// standard deviation of noise added to rotations for hand
  double hand_trans_stddev_; /// standard deviation of noise added to translations for hand
  double eye_rot_stddev_; /// standard deviation of noise added to rotations for eye
  double eye_trans_stddev_; /// standard deviation of noise added to translations for eye
  
  /** returns a random rotation matrix with the rotations around the three axis normally distributed around zero with a standard deviation of _stddev
   */
  Eigen::Matrix3d rotationNoise( double _stddev );
  
  /** returns a random vector with each component normally distributed around zero with a standard deviation of _stddev
   */
  Eigen::Vector3d translationNoise( double _stddev );
  
  std::vector<TransformationEstimator::PoseData> pose_pairs_; // to save generated pose pairs
};