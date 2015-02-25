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

#include "hand_eye_calibration/estimation_data.h"
#include "utils/ros_eigen.h"

using namespace Eigen;

TransformationEstimator::EstimationData::EstimationData( std::string _method_name, Eigen::Quaterniond _rot_EH, Eigen::Vector3d _E_trans_EH ):
  has_reprojection_error_(false),
  has_transformation_error_(false),
  method_name_(_method_name),
  rot_EH_(_rot_EH),
  E_trans_EH_(_E_trans_EH)
{
  
}


std::string TransformationEstimator::EstimationData::methodName()
{
  return method_name_;
}




geometry_msgs::Pose TransformationEstimator::EstimationData::getHandToEye()
{  
  geometry_msgs::Pose estimatedTransformation;
  
  estimatedTransformation.position.x = E_trans_EH_[0];
  estimatedTransformation.position.y = E_trans_EH_[1];
  estimatedTransformation.position.z = E_trans_EH_[2];
  estimatedTransformation.orientation.x = rot_EH_.x();
  estimatedTransformation.orientation.y = rot_EH_.y();
  estimatedTransformation.orientation.z = rot_EH_.z();
  estimatedTransformation.orientation.w = rot_EH_.w();
  
  return estimatedTransformation;
}


Eigen::Matrix3d TransformationEstimator::EstimationData::rotH2E()
{
  return rot_EH_.toRotationMatrix();
}

Eigen::Quaterniond TransformationEstimator::EstimationData::rot_EH()
{
  return rot_EH_;
}


Eigen::Matrix3d TransformationEstimator::EstimationData::rotE2H()
{
  return rot_EH_.conjugate().toRotationMatrix();
}

Eigen::Quaterniond TransformationEstimator::EstimationData::rot_HE()
{
  return rot_EH_.conjugate();
}


Eigen::Vector3d TransformationEstimator::EstimationData::transH2E()
{
  return E_trans_EH_;
}

Eigen::Vector3d TransformationEstimator::EstimationData::E_trans_EH()
{
  return transH2E();
}


Eigen::Vector3d TransformationEstimator::EstimationData::transE2H()
{
  return -(rotE2H()*E_trans_EH_);
}

Eigen::Vector3d TransformationEstimator::EstimationData::H_trans_HE()
{
  return transE2H();
}


Eigen::Matrix<double,4,4> TransformationEstimator::EstimationData::matrixH2E()
{
  Matrix<double,4,4> mH2E;
  
  Vector3d E_t_EH = E_trans_EH_;
  
  mH2E<<rotH2E(), E_t_EH, 0, 0, 0, 1;
  
  return mH2E;
}


Eigen::Matrix<double,4,4> TransformationEstimator::EstimationData::matrixE2H()
{
  Matrix<double,4,4> mE2H;
  
  Vector3d H_t_HE = -( rotE2H()*E_trans_EH_ );
  
  mE2H<<rotE2H(), H_t_HE, 0, 0, 0, 1;
  
  return mE2H;
}


void TransformationEstimator::EstimationData::setReprojectionError( st_is::StdError& _error )
{
  has_reprojection_error_ = true;
  reprojection_error_ = _error;
}


void TransformationEstimator::EstimationData::setTransformationErrors( TransformationError& _error )
{
  has_transformation_error_ = true;
  transformation_error_ = _error;
}


void TransformationEstimator::EstimationData::setTransformationErrors( st_is::StdError& _euler_angle_error, st_is::StdError& _relative_translation_error )
{
  has_transformation_error_ = true;
  transformation_error_.euler_angle_error = _euler_angle_error;
  transformation_error_.relative_translation_error = _relative_translation_error;
}


void TransformationEstimator::EstimationData::setTransformationErrors( std::pair<st_is::StdError,st_is::StdError>& _errors )
{
  setTransformationErrors(_errors.first,_errors.second);
}


bool TransformationEstimator::EstimationData::reprojectionError( st_is::StdError& _reprojection_error )
{
  if(!has_reprojection_error_) return false;
  
  _reprojection_error = reprojection_error_;
  return true;
}


bool TransformationEstimator::EstimationData::transformationError( TransformationError& _transformation_error )
{
  if(!has_transformation_error_) return false;
  
  _transformation_error = transformation_error_;
  return true;
}


bool TransformationEstimator::EstimationData::eulerAngleError( st_is::StdError& _euler_angle_error )
{
  if(!has_transformation_error_) return false;
  
  _euler_angle_error = transformation_error_.euler_angle_error;
  return true;
}

bool TransformationEstimator::EstimationData::relativeTranslationError( st_is::StdError& _relative_translation_error )
{
  if(!has_transformation_error_) return false;
  
  _relative_translation_error = transformation_error_.relative_translation_error;
  return true;
}