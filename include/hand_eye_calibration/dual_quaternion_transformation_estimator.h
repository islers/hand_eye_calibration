 
 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2014
*
*
* Class to estimate the transformation from a robot link (hand) to a camera 
* sensor frame (eye) [Hand-Eye-Calibration] using the method described in
* "Hand-Eye Calibration Using Dual Quaternions" by Konstantinos Daniilidis.
* It features a simple command line interface, the estimated transformations
* are printed to the command line as well.
* 
* subscribes:	- /hec/eye_position [geometry_msgs/Pose]: transformation grid->camera [rotation R_CG, position of origin of G in C
* 		- /hec/hand_position [geometry_msgs/Pose]: transformation hand->base  [rotation R_BH, position of origin of H in B
* 
* 
* Released under the GNU Lesser General Public License v3 (LGPLv3), see www.gnu.org
*
******************************************************************************/
 
 
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

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>


class DualQuaternionTransformationEstimator
{
  public:
    DualQuaternionTransformationEstimator( ros::NodeHandle* _n );
    ~DualQuaternionTransformationEstimator();
    
    /** adds the next eye- and the next hand-position to the list of
     * pose pairs which will be used for estimation and calculates a new transformation estimate which is stored in the respective vector*/
    void addLastRetrievedPosePair();
    
    /** deletes the last added pose pair */
    void deleteLastAddedPosePair();
    
    /** calculates the transformation estimate */
    void calculateTransformation(bool _suppressWarnings=false );
    
    /** returns the calculated transformation */
    geometry_msgs::Pose getHandToEye();
    
    /** returns the rotation matrix R_EH */
    Eigen::Matrix3d rotH2E();
    
    /** returns the rotation matrix R_HE */
    Eigen::Matrix3d rotE2H();
    
    /** returns the translation vector E_t_EH (position of H in E)*/
    Eigen::Vector3d transH2E();
    
    /** returns the translation vector H_t_HE (position of E in H)*/
    Eigen::Vector3d transE2H();
    
    /** returns the transformation matrix H_EH from hand to eye coordinates */
    Eigen::Matrix<double,4,4> matrixH2E();
    
    /** returns the transformation matrix H_HE from eye to hand coordinates */
    Eigen::Matrix<double,4,4> matrixE2H();
    
    /** clears all data to restart recording */
    void clearAll();

    void handListening( const geometry_msgs::PoseConstPtr& _newPose );
    void eyeListening( const geometry_msgs::PoseConstPtr& _newPose );
    
    /** returns the number of pose pairs added so far */
    int count();
    
    
    /** saves the hand and eye poses to a file, using the opencv storage functionality. Returns true if no problems occured */
    bool printToFile( std::string fileName_ );
    
    /** loads hand and eye poses from a file, saved using the printToFile method: both must be saved like a OpenCV Mat matrix with size 7xNumberOfPoses, where indices 0...3 represent the rotation quaternion and 4..6 the translation vector and the number of poses must be equal for both. The name of the hand poses must be "handPoses", the one of the eye poses "eyePoses". Returns true if succesful.
     * 
     * If destroyOldData_ is set to true, any previous hand-eye correspondences are dropped. If it is false, the correspondences loaded from file are added to the ones already stored.
    */
    bool loadFromFile( std::string fileName_, bool destroyOldData_=false );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::Subscriber handSubscriber_;
    ros::Subscriber eyeSubscriber_;
    
    std::vector< std::pair<geometry_msgs::Pose, geometry_msgs::Pose> > posePairs_; //pairs: hand,cam
    
    /** simple solver for the quadratic equation a*x² + bx + c = 0
     *  Returns false if the roots are imaginary, otherwhise the two roots are stored in _roots - twice
     *  the same value if only one root exists.
     */
    bool roots( double _aCoeff, double _bCoeff, double _cCoeff, std::pair<double,double>& _roots );
        
    bool transformationCalculated_;
    bool handRecorded_, eyeRecorded_;
    ros::Time recordedHandTimeStamp_, recordedEyeTimeStamp_;
    
    Eigen::Quaterniond rot_EH_; // current estimated rotation from hand to eye
    Eigen::Vector3d E_trans_EH_; // current estimated position of H (hand) origin in E (eye) coordinates
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > rotationEstimates_EH_; // estimated rotations for data subsets
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > translationEstimates_E_t_EH_; //estimated translations for data subsets
    
    Eigen::Matrix3d crossProdMatrix( Eigen::Vector3d _vec );
    Eigen::Vector3d geometryToEigen( const geometry_msgs::Point& _vec );
    Eigen::Quaterniond geometryToEigen( const geometry_msgs::Quaternion& _quat );
    std::pair<Eigen::Quaterniond,Eigen::Quaterniond> dualQuaternion( Eigen::Quaterniond _rot, Eigen::Vector3d _trans );
    
    geometry_msgs::Pose bufferedHand_, bufferedEye_;
        
    ros::NodeHandle* rosNode_;
};