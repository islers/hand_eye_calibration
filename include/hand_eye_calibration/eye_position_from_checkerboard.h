 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2014
*
*
* This class estimates the camera position from a rectified camera image with
* a checkerboard for a calibrated camera using OpenCV. The image with the
* detected checkerboard pattern is published to provide a feedback for
* calibration purposes.
* 
* subscribes:	- /camera/image_rect [sensor_msgs/Image]: Rectified camera image, e.g. published by image_proc
* 		- /camera/camera_info [sensor_msgs/CameraInfo]: Camera info data, published by the camera driver
* 
* publishes:	- /hec/eye_position [geometry_msgs/Pose]: The transformation from
* 		  checkerboard coordinates (O) to camera(eye) coordinates (E): the rotation
* 		  R_EO represented by a quaternion and the translation vector E_t_EO,
* 		  that is the coordinates of the origin of O in E coordinates.
* 		  Therefore, a transformation O->E would be carried out through:
* 		  x_E = R_EO*x_O + E_t_EO
* 
* Data needed on the parameter server (use either the command line or a yaml file):
* 		- /hec/checkerboard/squares_per_column (nr of internal chkrbrd edges per column)
* 		- /hec/checkerboard/squares_per_row (nr of internal chckrbrd edges per row
* 		- /hec/checkerboard/square_size (the size of checkerboard fields in [m])
* 	optional: (Will be loaded from camera_info topic otherwise, but data set on parameter server has priority. If data is read from camera_info topic it uses the values given in the projection matrix, not those in the camera matrix.)
* 		- /camera/fx (intrinsic camera matrix data)
* 		- /camera/fy (")
* 		- /camera/cx (")
* 		- /camera/cy (")
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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

class EyePositionFromCheckerboard
{
  public:
    EyePositionFromCheckerboard( ros::NodeHandle* _n );
    ~EyePositionFromCheckerboard();
    
    /** starts the listening */
    void run();

    void imageLoader(  const sensor_msgs::ImageConstPtr& _newImage );
    void cameraInfoUpdate( const sensor_msgs::CameraInfoConstPtr& _newCamInfo );
  private:
    ros::Publisher posePublisher_;
    ros::Subscriber cameraStream_;
    ros::Subscriber cameraInfoSubscriber_;
    
    bool newImageLoaded_; // whether a new image has been loaded
    cv::Mat currentImage_;
    cv::Mat cameraMatrix_;
    cv::Mat distortionCoefficients_; // plumb_bob model
    cv::Size patternSize_;
    vector<cv::Point3f> objectPointCoordinates_; // coordinates of to-be-detected checkerboard points in checkerboard frame
    
    bool initSuccess_; // indicates successful initialization of all parameters
    
    bool init(); //initializes (loads) parameters from parameter server
        
    ros::NodeHandle* rosNode_;
}; 
