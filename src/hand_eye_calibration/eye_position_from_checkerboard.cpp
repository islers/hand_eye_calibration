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

using namespace std;
 
EyePositionFromCheckerboard::EyePositionFromCheckerboard( ros::NodeHandle* _n ):
camera_data_retrieved_(false),count(0)
{
  me_myself_and_i_ = boost::shared_ptr<EyePositionFromCheckerboard>(this);
  
  ros_node_ = _n;
  
  pose_publisher_ = ros_node_->advertise<geometry_msgs::Pose>("/hec/eye_position",10);
  camera_stream_ = ros_node_->subscribe("/camera/image_rect",1, &EyePositionFromCheckerboard::imageLoader, this );
  camera_info_subscriber_ = ros_node_->subscribe("/camera/camera_info",1,&EyePositionFromCheckerboard::cameraInfoUpdate, this );
    
  eye_position_info_server_ = ros_node_->advertiseService("hec/eye_node_info", &EyePositionFromCheckerboard::serviceCameraPoseInfoRequest, this );
  
  // launch eye position server with separate callback queue
  ros::AdvertiseServiceOptions eye_pos_server_ops = ros::AdvertiseServiceOptions::create<hand_eye_calibration::CameraPose>("hec/eye_pose", boost::bind(&EyePositionFromCheckerboard::serviceCameraPoseRequest, this, _1, _2), me_myself_and_i_, &pos_srv_queue_ );
  eye_position_server_ = ros_node_->advertiseService( eye_pos_server_ops );
  
  init_success_ = false;
  new_image_loaded_ = false;
  distortion_coefficients_ = cv::Mat();
  
  return;
}


EyePositionFromCheckerboard::~EyePositionFromCheckerboard()
{
  
}



void EyePositionFromCheckerboard::run()
{
  // asynchronous spinner to handle the separate pose service callbacks
  ros::AsyncSpinner srv_spinner(1, &pos_srv_queue_);
  srv_spinner.start();
  
  ros::Rate rate(1.0); // once per sec
  while( !init_success_ && ros_node_->ok() ) // initalize all needed parameters
  {
    init_success_ = init();
    if( !init_success_ && ros_node_->ok() ) rate.sleep();
  }
  
  int image_wait_counter=0;
  int chessboard_wait_counter=0; // only show message that chessboard wasn't found if it has been so for a while;
  while( ros_node_->ok() )
  {
    //ros::spinOnce();    
    ImageState image_state = processImageIfAvailable();
    if( image_state != NO_NEW_IMAGE_AVAILABLE ) // if an image is available
    {
      image_wait_counter=0;
            
      if( image_state == CHESSBOARD_FOUND )
      {
	  chessboard_wait_counter=0;
      }
      else
      {
	if( chessboard_wait_counter++ > 10 )
	{
	  chessboard_wait_counter=0;
	  ROS_INFO("Chessboard wasn't found in the given image");
	}
      }
      cv::imshow( "Checkerboard publisher", current_image_->image );
    }
    else
    {
      if( image_wait_counter++ > 30 )
      {
	ROS_WARN("EyePositionFromCheckerboard::run()::no new image available: waiting...");
	image_wait_counter=0;
      }
      //ros::Rate(30).sleep();
    }
    
    cv::waitKey(20); //wait x ms and let opencv display the mat
    ros::spinOnce();
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
  
  current_image_ = cv_ptr;
  
  new_image_loaded_ = true;
  last_image_retrieval_ = _newImage->header.stamp;
  
  return;
}



void EyePositionFromCheckerboard::cameraInfoUpdate( const sensor_msgs::CameraInfoConstPtr& _newCamInfo )
{
  camera_info_ = *_newCamInfo;
  
  double fx = _newCamInfo->P[0];//K[0];
  double fy = _newCamInfo->P[5];//K[4];
  double cx = _newCamInfo->P[2];//K[2];
  double cy = _newCamInfo->P[6];//K[5];
  
  if( _newCamInfo->distortion_model != "plumb_bob" )
  {
    ROS_ERROR("The distortion model of the published camera_info data is unknown. It is '%s' but should be 'plumb_bob'",_newCamInfo->distortion_model.c_str());
    return;
  }
  distortion_coefficients_ = cv::Mat(5, 1, CV_64F);
  
  for( int i=0;i<5;i++ ) distortion_coefficients_.at<double>(i) = _newCamInfo->D[i];
    
  camera_matrix_ = cv::Mat::zeros(3,3,CV_64FC1);
  camera_matrix_.at<double>(0,0) = fx;
  camera_matrix_.at<double>(1,1) = fy;
  camera_matrix_.at<double>(2,2) = 1;
  camera_matrix_.at<double>(0,2) = cx;
  camera_matrix_.at<double>(1,2) = cy;
  
  camera_data_retrieved_ = true;
  
  return;
}

bool EyePositionFromCheckerboard::serviceCameraPoseRequest( hand_eye_calibration::CameraPose::Request& _req, hand_eye_calibration::CameraPose::Response& _res )
{
  ROS_INFO_STREAM("Camera pose request service called.");
  ros::Time request_stamp = _req.request.request_stamp;
  ros::Duration max_wait_time = _req.request.max_wait_time;
  ros::Time request_time = ros::Time::now();
  count++;
  ros::Time time_limit = request_time+max_wait_time;
    
  ros::Rate rate(100.0); //Hz
  ros::Time last_used_image_time = request_time;
  while( ros::Time::now() <= time_limit )
  {
    { // mutex scope
      boost::mutex::scoped_lock scoped_lock(checkerboard_image_mutex_);
      if( last_image_retrieval_with_chkrbrd_ >= request_time ) // new checkerboard image retrieved after service call
      {
	ROS_INFO_STREAM("Chessboard pattern found.");
	_res.description.stamp = ros::Time::now();
	_res.description.request_stamp = request_stamp;
	_res.description.pose_found = true;
	
	std::vector<hand_eye_calibration::Point2D> checkerboard_corners;
	for( unsigned int i = 0; i < checkerboard_corner_coordinates_->size(); i++ )
	{
	  hand_eye_calibration::Point2D point;
	  point.x = (*checkerboard_corner_coordinates_)[i].x;
	  point.y = (*checkerboard_corner_coordinates_)[i].y;
	  checkerboard_corners.push_back( point );
	}
	_res.description.point_coordinates = checkerboard_corners;
		
	_res.description.pose = last_checkerboard_pose_;
	switch(count)
	{
	  case 1:
	  {
	    Eigen::Matrix<double,3,4> eye_pose;
	    eye_pose<<1,0,0,4, 0,1,0,3, 0,0,1,0;
	    _res.description.pose = st_is::geometryPose(eye_pose);
	    break;
	  }
	  case 2:
	  {
	    Eigen::Matrix<double,3,4> eye_pose;
	    eye_pose<<0,1,0,0, 1,0,0,0, 0,0,-1,8;
	    _res.description.pose = st_is::geometryPose(eye_pose);
	    break;
	  }
	  case 3:
	  {
	    Eigen::Matrix<double,3,4> eye_pose;
	    eye_pose<<0,0,1,-2, 0,1,0,-3, -1,0,0,5;
	    _res.description.pose = st_is::geometryPose(eye_pose);
	    break;
	  }
	};
	
	last_checkerboard_image_->toImageMsg(_res.description.image);
	
	ROS_INFO_STREAM("Called successfully.");	
	return true;
      }
      else
      {
	//ROS_INFO("No checkerboard image available yet.");
      }
    }
    //ROS_INFO("EyePositionFromCheckerboard::serviceCameraPoseRequest::called, waiting for new image");
    //processImageIfAvailable();
    //ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("hand_eye_camera_pose service was called but unable to retrieve an image frame within the given time limit");
  // time exceeded, failed to extract new pose
  _res.description.stamp = ros::Time::now();
  _res.description.request_stamp = request_stamp;
  _res.description.pose_found = false;
  
  return true;
}

bool EyePositionFromCheckerboard::serviceCameraPoseInfoRequest( hand_eye_calibration::CameraPoseInfo::Request& _req, hand_eye_calibration::CameraPoseInfo::Response& _res )
{
  _res.info.camera_info = camera_info_;
  
  std::vector<geometry_msgs::Point> pattern_coordinates;
  for( unsigned int i=0; i<object_point_coordinates_.size(); i++ )
  {
    geometry_msgs::Point point;
    point.x = object_point_coordinates_[i].x;
    point.y = object_point_coordinates_[i].y;
    point.z = object_point_coordinates_[i].z;
    pattern_coordinates.push_back( point );
  }
  _res.info.pattern_coordinates = pattern_coordinates;
  return true;
}

EyePositionFromCheckerboard::ImageState EyePositionFromCheckerboard::processImageIfAvailable()
{
  if( !current_image_->image.empty() && new_image_loaded_ ) // if an image is available
    {
      boost::shared_ptr< cv::vector<cv::Point2f> > chkbrdCorners(new cv::vector<cv::Point2f>() );
      bool chessboardFound = calculateChessboardCorners( current_image_->image, *chkbrdCorners );
      
      
      if( chessboardFound )
      {
	  boost::mutex::scoped_lock scoped_lock(checkerboard_image_mutex_);
	  cv::Mat rotation_vector, translation_vector;
	  	  
	  calculatePose( *chkbrdCorners, rotation_vector, translation_vector );
	  
	  geometry_msgs::Pose cameraPose = geometryPoseFromVectors( rotation_vector, translation_vector );
	  
	  cout<<endl<<"New camera pose found:"<<endl;
	  cout<<endl<<"The translation vector is:"<<endl<<translation_vector<<endl<<endl<<"which has a length of "<<cv::norm( translation_vector, cv::NORM_L2 )<<" m."<<endl;
	  cout<<endl<<"The rotation vector is "<<endl<<rotation_vector<<endl;
	  
	  // save last "checkerboard state"
	  last_image_retrieval_with_chkrbrd_ = last_image_retrieval_;
	  checkerboard_corner_coordinates_ = chkbrdCorners;
	  last_checkerboard_pose_ = cameraPose;
	  last_checkerboard_image_ = current_image_;
	  current_image_->image.copyTo( last_checkerboard_image_->image ); //enforce deep copy
	  
	  pose_publisher_.publish( cameraPose );
	  
	  drawChessboardCorners( current_image_->image, pattern_size_, *chkbrdCorners, chessboardFound );
	  return CHESSBOARD_FOUND;
      }
      else
      {
	return NO_CHESSBOARD_FOUND;
      }
      
      new_image_loaded_ = false;
    }
    else return NO_NEW_IMAGE_AVAILABLE;
}

bool EyePositionFromCheckerboard::calculateChessboardCorners( cv::Mat& _image, cv::vector<cv::Point2f>& _chkbrdCorners )
{
  bool chessboardFound = cv::findChessboardCorners( _image, pattern_size_, _chkbrdCorners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK );
  
  // refine the positions of the found corners
  if( chessboardFound )
  {
    cv::Mat grayImage;
    cv::cvtColor( _image, grayImage, CV_BGR2GRAY);
    cv::cornerSubPix( grayImage, _chkbrdCorners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ));
    return true;
  }
  
  return false;
}

void EyePositionFromCheckerboard::calculatePose( cv::vector<cv::Point2f>& _chkbrdCorners, cv::Mat& _rotation_vector, cv::Mat& _translation_vector )
{
  
  // finds object pose from 3D-2D point correspondences: tvecs: position of the object origin in camera coordinates, rotation_vector: represents R_CO (rotation matrix from object to camera coordinates)
  cv::solvePnP( object_point_coordinates_, _chkbrdCorners, camera_matrix_, distortion_coefficients_, _rotation_vector, _translation_vector );
  return;
}

geometry_msgs::Pose EyePositionFromCheckerboard::geometryPoseFromVectors( cv::Mat& _rotation_vector, cv::Mat& _translation_vector )
{
  // create the ROS message
	  
  geometry_msgs::Pose cameraPose;
  cameraPose.position.x = _translation_vector.at<double>(0);
  cameraPose.position.y = _translation_vector.at<double>(1);
  cameraPose.position.z = _translation_vector.at<double>(2);
  
  // calculate quaternion for rotation from rotation vector
  double thetaHalf = cv::norm( _rotation_vector, cv::NORM_L2 )/2;
  double scale = 0.5/thetaHalf;
	  
  double sinThetaHalfScaled = sin(thetaHalf)*scale;
  cameraPose.orientation.x = _rotation_vector.at<double>(0)*sinThetaHalfScaled;
  cameraPose.orientation.y = _rotation_vector.at<double>(1)*sinThetaHalfScaled;
  cameraPose.orientation.z = _rotation_vector.at<double>(2)*sinThetaHalfScaled;
  cameraPose.orientation.w = cos(thetaHalf);
  
  return cameraPose;
}


bool EyePositionFromCheckerboard::init()
{
  double sPc, sPr, squareSize;
  
  ros::Rate rate(1);
  bool initialized;
  do
  {
    ros::spinOnce();
    initialized = true;
    
    ROS_INFO("Waiting for all parameters to be initialized...");
    
    if( !camera_data_retrieved_ )
    {
      initialized =  false;
    }
    if( !ros_node_->getParam("/hec/checkerboard/squares_per_column",sPc) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing squares_per_column parameter on /hec/checkerboard/squares_per_column");
      initialized =  false;
    }
    if( !ros_node_->getParam("/hec/checkerboard/squares_per_row",sPr) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing squares_per_row parameter on /hec/checkerboard/squares_per_row");
      initialized =  false;
    }
    if( !ros_node_->getParam("/hec/checkerboard/square_size",squareSize) )
    {
      ROS_WARN("EyePositionFromCheckerboard::initialization failed - missing square_size parameter on /hec/checkerboard/square_size");
      initialized =  false;
    }
    if(!initialized) rate.sleep();
  }while( !initialized && ros_node_->ok() );
  
  camera_info_subscriber_.shutdown();
  
  
  pattern_size_ = cv::Size( sPr, sPc );
  
  /* coordinates of the checkerboard points in checkerboard coordinate system (in which the pose will be estimated)
   the OpenCV findChessboardCorners-function orders the point sequentially in an array row by row, left to right in every row.
   x-axis along rows, y-axis along columns, z-axis perpendicular to it */
  
  object_point_coordinates_;
  for( int i=0; i<pattern_size_.height; i++ )
  {
    for( int j=0; j<pattern_size_.width; j++ )
    {
      object_point_coordinates_.push_back( cv::Point3f( j*squareSize, i*squareSize, 0 ) ); // x,y,z
    }
  }
  
  
  return true;
}
