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

#include "hand_eye_calibration/CameraPose.h"
#include "hand_eye_calibration/CameraPoseInfo.h"


class EyePositionFromCheckerboard
{
  public:
    EyePositionFromCheckerboard( ros::NodeHandle* _n );
    ~EyePositionFromCheckerboard();
    
    /** starts the listening */
    void run();

    void imageLoader(  const sensor_msgs::ImageConstPtr& _newImage );
    void cameraInfoUpdate( const sensor_msgs::CameraInfoConstPtr& _newCamInfo );
    bool serviceCameraPoseRequest( hand_eye_calibration::CameraPose::Request& _req, hand_eye_calibration::CameraPose::Response& _res );
    bool serviceCameraPoseInfoRequest( hand_eye_calibration::CameraPoseInfo::Request& _req, hand_eye_calibration::CameraPoseInfo::Response& _res );
  private:
    ros::Publisher posePublisher_;
    ros::Subscriber cameraStream_;
    ros::Subscriber cameraInfoSubscriber_;
    ros::ServiceServer eyePositionServer_;
    ros::ServiceServer eyePositionInfoServer_;
    
    /// locates the checkerboard corners in the given image
    /** The function locates the checkerboard corners in the given image, writing their coordinates
     * in the array given. It returns true if the checkerboard pattern was found
     * @param _image The image in which the pattern should be searched for
     * @param _chkbrdCorners array in which the coordinates will be written
     * @return bool true if the checkerboard pattern was found
     */
    bool calculateChessboardCorners( cv::Mat& _image, cv::vector<cv::Point2f>& _chkbrdCorners );
    
    /// extracts a pose from extracted checkerboard pattern coordinates
    /** finds object pose from 3D-2D point correspondences
     * @param _chkbrdCorners extracted 2d checkerboard pattern coordinates
     * @param _rotation_vector represents R_CO (rotation matrix from object to camera coordinates)
     * @param _translation_vector position of the object origin in camera coordinates
     */
    void calculatePose( cv::vector<cv::Point2f>& _chkbrdCorners, cv::Mat& _rotation_vector, cv::Mat& _translation_vector );
    
    /// calculates a gemoetry_msgs::Pose structure from given rotation and translation vectors
    /**
     * @param _rotation_vector represents R_CO (rotation matrix from object to camera coordinates)
     * @param _translation_vector position of the object origin in camera coordinates
     */
    geometry_msgs::Pose geometryPoseFromVectors( cv::Mat& _rotation_vector, cv::Mat& _translation_vector );
    
    bool newImageLoaded_; // whether a new image has been loaded
    ros::Time lastImageRetrieval_; // time stamp of the last retrieved image
    
    cv_bridge::CvImage currentImage_;
    bool camera_data_retrieved_;
    cv::Mat cameraMatrix_;
    sensor_msgs::CameraInfo camera_info_;
    cv::Mat distortionCoefficients_; // plumb_bob model
    cv::Size patternSize_;
    std::vector<cv::Point3f> objectPointCoordinates_; // coordinates of to-be-detected checkerboard points in checkerboard frame
    
    bool initSuccess_; // indicates successful initialization of all parameters
    
    bool init(); //initializes (loads) parameters from parameter server
        
    ros::NodeHandle* rosNode_;
}; 
