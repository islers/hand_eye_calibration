#include "hand_eye_calibration/EyePositionFromCheckerboard.hpp"
 
EyePositionFromCheckerboard::EyePositionFromCheckerboard( ros::NodeHandle* _n )
{
  rosNode_ = _n;
  
  posePublisher_ = rosNode_->advertise<geometry_msgs::Pose>("/hec/eye_position",10);
  cameraStream_ = rosNode_->subscribe("/camera/image_rect",1, &EyePositionFromCheckerboard::imageLoader, this );
  
  initSuccess_ = false;
  newImageLoaded_ = false;
  
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
	  cv::solvePnP( objectPointCoordinates_, chkbrdCorners, cameraMatrix_, cv::Mat(), rotation_vector, translation_vector );
	  	  
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
      ros::Rate(1).sleep();
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


bool EyePositionFromCheckerboard::init()
{
  double fx, fy, cx, cy, sPc, sPr, squareSize;
  if( !rosNode_->getParam("/camera/fx",fx) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing fx parameter on /camera/fx");
    return false;
  }
  if( !rosNode_->getParam("/camera/fy",fy) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing fy parameter on /camera/fy");
    return false;
  }
  if( !rosNode_->getParam("/camera/cx",cx) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing cx parameter on /camera/cx");
    return false;
  }
  if( !rosNode_->getParam("/camera/cy",cy) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing cy parameter on /camera/cy");
    return false;
  }
  if( !rosNode_->getParam("/hec/checkerboard/squares_per_column",sPc) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing squares_per_column parameter on /hec/checkerboard/squares_per_column");
    return false;
  }
  if( !rosNode_->getParam("/hec/checkerboard/squares_per_row",sPr) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing squares_per_row parameter on /hec/checkerboard/squares_per_row");
    return false;
  }
  if( !rosNode_->getParam("/hec/checkerboard/square_size",squareSize) )
  {
    ROS_ERROR("EyePositionFromCheckerboard::initialization failed - missing square_size parameter on /hec/checkerboard/square_size");
    return false;
  }
  
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