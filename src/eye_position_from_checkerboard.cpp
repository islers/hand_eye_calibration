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

#include "hand_eye_calibration/eye_position_from_checkerboard.h"
 
EyePositionFromCheckerboard::EyePositionFromCheckerboard( ros::NodeHandle* _n )
{
  rosNode_ = _n;
  
  posePublisher_ = rosNode_->advertise<geometry_msgs::Pose>("/hec/eye_position",10);
  cameraStream_ = rosNode_->subscribe("/camera/image_rect",1, &EyePositionFromCheckerboard::imageLoader, this );
  cameraInfoSubscriber_ = rosNode_->subscribe("/camera/camera_info",1,&EyePositionFromCheckerboard::cameraInfoUpdate, this );
  
  initSuccess_ = false;
  newImageLoaded_ = false;
  distortionCoefficients_ = cv::Mat();
  
  return;
}


EyePositionFromCheckerboard::~EyePositionFromCheckerboard()
{
  
}



void EyePositionFromCheckerboard::run()
{
  ros::Rate rate(1.0); // once per sec
  while( !initSuccess_ && rosNode_->ok() ) // initalize all needed parameters
  {
    initSuccess_ = init();
    if( !initSuccess_ && rosNode_->ok() ) rate.sleep();
  }
  
  
  while( rosNode_->ok() )
  {
    ros::spinOnce();
    
    if( !currentImage_.empty() && newImageLoaded_ ) // if an image is available
    {
      cv::vector<cv::Point2f> chkbrdCorners;
            
      bool chessboardFound = cv::findChessboardCorners( currentImage_, patternSize_, chkbrdCorners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK );
      
      // refine the positions of the found corners
      if( chessboardFound )
      {
	cv::Mat grayImage;
	cv::cvtColor( currentImage_, grayImage, CV_BGR2GRAY);
	cv::cornerSubPix( grayImage, chkbrdCorners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ));
	drawChessboardCorners( currentImage_, patternSize_, chkbrdCorners, chessboardFound );
      }
      
      cv::imshow( "Checkerboard corner detection", currentImage_ );
      
      
      if( chessboardFound )
      {
	  cv::Mat rotation_vector, translation_vector;
	  
	  // finds object pose from 3D-2D point correspondences: tvecs: position of the object origin in camera coordinates, rotation_vector: represents R_CO (rotation matrix from object to camera coordinates)
	  cv::solvePnP( objectPointCoordinates_, chkbrdCorners, cameraMatrix_, distortionCoefficients_, rotation_vector, translation_vector );
	  	  
	  // create the ROS message
	  
	  geometry_msgs::Pose cameraPose;
	  cameraPose.position.x = translation_vector.at<double>(0);
	  cameraPose.position.y = translation_vector.at<double>(1);
	  cameraPose.position.z = translation_vector.at<double>(2);
	  
	  // calculate quaternion for rotation from rotation vector
	  double thetaHalf = cv::norm( rotation_vector, cv::NORM_L2 )/2;
	  double scale = 0.5/thetaHalf;
	  	  
	  double sinThetaHalfScaled = sin(thetaHalf)*scale;
	  cameraPose.orientation.x = rotation_vector.at<double>(0)*sinThetaHalfScaled;
	  cameraPose.orientation.y = rotation_vector.at<double>(1)*sinThetaHalfScaled;
	  cameraPose.orientation.z = rotation_vector.at<double>(2)*sinThetaHalfScaled;
	  cameraPose.orientation.w = cos(thetaHalf);
	  
	  cout<<endl<<"New camera pose found:"<<endl;
	  cout<<endl<<"The translation vector is:"<<endl<<translation_vector<<endl<<endl<<"which has a length of "<<cv::norm( translation_vector, cv::NORM_L2 )<<" m."<<endl;
	  cout<<endl<<"The rotation vector is "<<endl<<rotation_vector<<endl;
	  
	  posePublisher_.publish( cameraPose );
      }
      else
      {
	ROS_INFO("Chessboard wasn't found in the given image");
      }
      
      newImageLoaded_ = false;
      
      cv::waitKey(20); //wait x ms and let opencv display the mat
    }
    else
    {
      ROS_WARN("EyePositionFromCheckerboard::run()::no new image available: waiting...");
      ros::Rate(30).sleep();
    }
  }
  return;
}



void EyePositionFromCheckerboard::imageLoader(  const sensor_msgs::ImageConstPtr& _newImage )
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(_newImage, sensor_msgs::image_encodings::BGR8);
  }
  catch( cv_bridge::Exception& e )
  {
    ROS_ERROR("EyePositionFromCheckerboard::imageLoader::cv_bridge exception: %s",e.what() );
    return;
  }
  
  currentImage_ = cv_ptr->image;
  
  newImageLoaded_ = true;
  
  return;
}



void EyePositionFromCheckerboard::cameraInfoUpdate( const sensor_msgs::CameraInfoConstPtr& _newCamInfo )
{
  double fx = _newCamInfo->P[0];//K[0];
  double fy = _newCamInfo->P[5];//K[4];
  double cx = _newCamInfo->P[2];//K[2];
  double cy = _newCamInfo->P[6];//K[5];
  
  if( _newCamInfo->distortion_model != "plumb_bob" )
  {
    ROS_ERROR("The distortion model of the published camera_info data is unknown. It is '%s' but should be 'plumb_bob'",_newCamInfo->distortion_model.c_str());
    return;
  }
  distortionCoefficients_ = cv::Mat(5, 1, CV_64F);
  
  for( int i=0;i<5;i++ ) distortionCoefficients_.at<double>(i) = _newCamInfo->D[i];
  
  
  if( !rosNode_->getParam("/camera/fx",fx) ) rosNode_->setParam("/camera/fx",fx);
  if( !rosNode_->getParam("/camera/fy",fy) ) rosNode_->setParam("/camera/fy",fy);
  if( !rosNode_->getParam("/camera/cx",cx) ) rosNode_->setParam("/camera/cx",cx);
  if( !rosNode_->getParam("/camera/cy",cy) ) rosNode_->setParam("/camera/cy",cy);
  
  return;
}


bool EyePositionFromCheckerboard::init()
{
  double fx, fy, cx, cy, sPc, sPr, squareSize;
  
  ros::Rate rate(1);
  bool initialized;
  do
  {
    ros::spinOnce();
    initialized = true;
    
    cout<<endl<<"Load parameters from server..."<<endl;
    
    if( !rosNode_->getParam("/camera/fx",fx) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing fx parameter on /camera/fx");
      initialized =  false;
    }
    if( !rosNode_->getParam("/camera/fy",fy) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing fy parameter on /camera/fy");
      initialized =  false;
    }
    if( !rosNode_->getParam("/camera/cx",cx) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing cx parameter on /camera/cx");
      initialized =  false;
    }
    if( !rosNode_->getParam("/camera/cy",cy) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing cy parameter on /camera/cy");
      initialized =  false;
    }
    if( !rosNode_->getParam("/hec/checkerboard/squares_per_column",sPc) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing squares_per_column parameter on /hec/checkerboard/squares_per_column");
      initialized =  false;
    }
    if( !rosNode_->getParam("/hec/checkerboard/squares_per_row",sPr) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing squares_per_row parameter on /hec/checkerboard/squares_per_row");
      initialized =  false;
    }
    if( !rosNode_->getParam("/hec/checkerboard/square_size",squareSize) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing square_size parameter on /hec/checkerboard/square_size");
      initialized =  false;
    }
    if(!initialized) rate.sleep();
  }while( !initialized && rosNode_->ok() );
  
  cameraInfoSubscriber_.shutdown();
  
  cameraMatrix_ = cv::Mat::zeros(3,3,CV_64FC1);
  cameraMatrix_.at<double>(0,0) = fx;
  cameraMatrix_.at<double>(1,1) = fy;
  cameraMatrix_.at<double>(2,2) = 1;
  cameraMatrix_.at<double>(0,2) = cx;
  cameraMatrix_.at<double>(1,2) = cy;
  
  patternSize_ = cv::Size( sPr, sPc );
  
  /* coordinates of the checkerboard points in checkerboard coordinate system (in which the pose will be estimated)
   the OpenCV findChessboardCorners-function orders the point sequentially in an array row by row, left to right in every row.
   x-axis along rows, y-axis along columns, z-axis perpendicular to it */
  
  objectPointCoordinates_;
  for( int i=0; i<patternSize_.height; i++ )
  {
    for( int j=0; j<patternSize_.width; j++ )
    {
      objectPointCoordinates_.push_back( cv::Point3f( j*squareSize, i*squareSize, 0 ) ); // x,y,z
    }
  }
  
  
  return true;
}
