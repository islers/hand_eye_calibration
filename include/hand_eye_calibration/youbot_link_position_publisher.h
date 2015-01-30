 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2014
*
*
* This class reads tf arm joint transformations published on the /tf topic by the
* KUKA youbot controllers and republishes the transformation from the base to the last
* joint (joint 5) on /hec/hand_position as geometry_msgs/Pose type.
* 
* subscribes:	- /tf [expecting tf2_msgs/TFMessage]
* 
* publishes:	- /hec/eye_position [geometry_msgs/Pose]: default is transformation link 0 (arm base) ->link 5
* 		 thus: rotation R_BH (rotation hand->base) and translation t_BH in B coordinates
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
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hand_eye_calibration/HandPose.h"


class YoubotLinkPositionPublisher
{
  public:
    /** constructor expects as arguments the id's of the arm links for which the transformation shall be published:
     * 0: arm_base (rigidly fixed to youbot base)
     * 1-5: the moving arm links
     */
    YoubotLinkPositionPublisher( ros::NodeHandle* _n, int _sourceId=0, int _targetId=5 );
    ~YoubotLinkPositionPublisher();
    
    /** starts the listening */
    void run();

    bool serviceHandPoseRequest( hand_eye_calibration::HandPose::Request& _req, hand_eye_calibration::HandPose::Response& _res );
  private:
    ros::Publisher posePublisher_;
    ros::ServiceServer hand_position_server_;
    
    int sourceFrameId_, targetFrameId_;
    std::vector<std::string> linkNames_;
    
    /// calculates the transformation between the specified links
    /**
     * @param _hand_pose the calculated transformation
     * @return bool true if the transformation was successfully calculated
     */
    bool calculateTransformation( geometry_msgs::Pose& _hand_pose );
    
    tf2_ros::Buffer tfCore_; // object that can be queried for specific transformations
    tf2_ros::TransformListener tf2Listener_;
        
    ros::NodeHandle* rosNode_;
};