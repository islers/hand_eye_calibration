 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2015
*
*
* This class reads tf arm joint transformations between the links specified on
* the parameter server and republishes them on the /hec/eye_position topic along with advertising it as a service.
* 
* subscribes:	- /tf [expecting tf2_msgs/TFMessage]
* 
* publishes:	- /hec/eye_position [geometry_msgs/Pose]: transformation base_link ->end_link (hand)
* 		 thus: rotation R_BH (rotation hand->base) and translation t_BH in B coordinates
* services provided:
* 	- hec_hand_pose [HandPose.srv]
* 
* expects on parameter server:
* 	- hec/hand/base_link
* 	- hec/hand/end_link
* 
* 
* Released under the GNU Lesser General Public License v3 (LGPLv3), see www.gnu.org
*
******************************************************************************/
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

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hand_eye_calibration/HandPose.h"


class TF2Republisher
{
  public:
    /** constructor expects as arguments the id's of the arm links for which the transformation shall be published:
     * 0: arm_base (rigidly fixed to youbot base)
     * 1-5: the moving arm links
     */
    TF2Republisher( ros::NodeHandle* _n );
    ~TF2Republisher();
    
    /** starts the listening */
    void run();

    bool serviceHandPoseRequest( hand_eye_calibration::HandPose::Request& _req, hand_eye_calibration::HandPose::Response& _res );
  private:
    ros::Publisher pose_publisher_;
    ros::ServiceServer hand_position_server_;
        
    std::string base_link_, end_link_;
    
    /// calculates the transformation between the specified links
    /**
     * @param _hand_pose the calculated transformation
     * @return bool true if the transformation was successfully calculated
     */
    bool calculateTransformation( geometry_msgs::Pose& _hand_pose );
    
    tf2_ros::Buffer tf_core_; // object that can be queried for specific transformations
    tf2_ros::TransformListener tf2_listener_;
        
    ros::NodeHandle* ros_node_;
};