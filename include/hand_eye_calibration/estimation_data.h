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

#include "hand_eye_calibration/transformation_estimator.h"

struct TransformationEstimator::TransformationError
{
  st_is::StdError euler_angle_error;
  st_is::StdError relative_translation_error;
};

class TransformationEstimator::EstimationData
{
  public:
    ///transformation error datatype
    
    EstimationData( std::string _method_name, Eigen::Quaterniond _rot_EH, Eigen::Vector3d _E_trans_EH );
    
    
    /** returns the name of the method used to obtain the estimation
     */
    std::string methodName();
    
    
    /** returns the calculated transformation */
    geometry_msgs::Pose getHandToEye();
    
    /** returns the rotation matrix R_EH */
    Eigen::Matrix3d rotH2E();
    Eigen::Quaterniond rot_EH();
    
    /** returns the rotation matrix R_HE */
    Eigen::Matrix3d rotE2H();
    Eigen::Quaterniond rot_HE();
    
    /** returns the translation vector E_t_EH (position of H in E)*/
    Eigen::Vector3d transH2E();
    Eigen::Vector3d E_trans_EH();
    
    /** returns the translation vector H_t_HE (position of E in H)*/
    Eigen::Vector3d transE2H();
    Eigen::Vector3d H_trans_HE();
    
    /** returns the transformation matrix H_EH from hand to eye coordinates */
    Eigen::Matrix<double,4,4> matrixH2E();
    
    /** returns the transformation matrix H_HE from eye to hand coordinates */
    Eigen::Matrix<double,4,4> matrixE2H();
    
    
    void setReprojectionError( st_is::StdError& _error );
    void setTransformationErrors( TransformationError& _error );
    void setTransformationErrors( st_is::StdError& _euler_angle_rms_error, st_is::StdError& _relative_translation_error );
    void setTransformationErrors( std::pair<st_is::StdError,st_is::StdError>& _errors ); // pair<euler_angle_rms_error,relative_translation_error>
    
    /// returns true if a reprojection_error is available and writes the value to the argument
    bool reprojectionError( st_is::StdError& _reprojection_error );
    /// returns true if a transformation error is available and writes the value to the argument
    bool transformationError( TransformationError& _transformation_error );
    /// returns true if a euler_angle_rms_error is available and writes the value to the argument
    bool eulerAngleError( st_is::StdError& _euler_angle_rms_error );
    /// returns true if a relative_translation_error is available and writes the value to the argument
    bool relativeTranslationError( st_is::StdError& _relative_translation_error );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:    
    // hand-eye estimation
    Eigen::Quaterniond rot_EH_;
    Eigen::Vector3d E_trans_EH_;
    
    std::string method_name_; // the name of the method used to obtain the estimate
    // error measures for the estimation
    bool has_reprojection_error_;
    st_is::StdError reprojection_error_;
    bool has_transformation_error_;
    TransformationError transformation_error_;
    
};