 
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

#include "hand_eye_calibration/transformation_estimator.h"


class DualQuaternionTransformationEstimator:public TransformationEstimator
{
  public:
    DualQuaternionTransformationEstimator( ros::NodeHandle* _n );
    ~DualQuaternionTransformationEstimator();    /** calculates the transformation estimate */
    
    void calculateTransformation(bool _suppressWarnings=false );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};