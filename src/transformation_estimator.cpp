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

#include "hand_eye_calibration/transformation_estimator.h" 
using namespace std;
using namespace Eigen;

TransformationEstimator::TransformationEstimator( ros::NodeHandle* _n ):
max_service_wait_time_(0,30000000)
{
  ros_node_ = _n;
  
  
  hand_recorded_ = false;
  eye_recorded_ = false;
  transformation_calculated_ = false;
}


TransformationEstimator::~TransformationEstimator()
{
  
}


bool TransformationEstimator::addNewPosePair()
{
  hand_eye_calibration::CameraPose cam_pose_request;
  hand_eye_calibration::HandPose hand_pose_request;
  
  ros::Time current_stamp = ros::Time::now();
  
  cam_pose_request.request.request.request_stamp = current_stamp;
  cam_pose_request.request.request.max_wait_time = max_service_wait_time_;
  hand_pose_request.request.request.request_stamp = current_stamp;
  hand_pose_request.request.request.max_wait_time = max_service_wait_time_;
  
  if( !ros::service::call("hec_eye_pose",cam_pose_request) )
  {
    ROS_ERROR("TransformationEstimator::addNewPosePair(): Unable to contact 'hec_eye_pose' service. Cannot add new pose pair.");
    return false;
  }
  else if( !ros::service::call("hec_hand_pose",hand_pose_request) )
  {
    ROS_ERROR("TransformationEstimator::addNewPosePair(): Unable to contact 'hec_hand_pose' service. Cannot add new pose pair.");
    return false;
  }
  // check if both poses were found
  if( !cam_pose_request.response.description.pose_found )
  {
     ROS_WARN("TransformationEstimator::addNewPosePair(): The contacted 'hec_eye_pose' service  was not able to locate the camera, no pose retrieved in time. Cannot add new pose pair.");
     return false;
  }
  else if( !hand_pose_request.response.description.pose_found )
  {
     ROS_WARN("TransformationEstimator::addNewPosePair(): The contacted 'hec_hand_pose' service was not able to calculate the location of the hand, no pose retrieved in time. Cannot add new pose pair.");
     return false;
  }
  // load camera pose information
  geometry_msgs::Pose cam_pose = cam_pose_request.response.description.pose;
  sensor_msgs::Image cam_image = cam_pose_request.response.description.image;
  std::vector<hand_eye_calibration::Point2D> cam_pattern_coordinates = cam_pose_request.response.description.point_coordinates;
 
  // load hand pose information
  geometry_msgs::Pose hand_pose = hand_pose_request.response.description.pose;
  
  pose_pairs_.push_back( std::pair<geometry_msgs::Pose, geometry_msgs::Pose>(hand_pose,cam_pose) );
  calib_pattern_image_coordinates_.push_back( cam_pattern_coordinates );
  
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_cam_image = (*cv_ptr).image;
    calibration_images_.push_back( cv_cam_image );
  }
  catch( cv_bridge::Exception& e )
  {
    ROS_WARN_STREAM("TransformationEstimator::addNewPosePair()::cv_bridge exception: "<<e.what()<<". No image is added." );
    calibration_images_.push_back( cv::Mat() );
  }
  
  if( pose_pairs_.size()>=2 ) //estimation is possible
  {
    calculateTransformation(true);
    
    EstimationData new_estimate;
    new_estimate.rot_EH = rot_EH_;
    new_estimate.E_trans_EH = E_trans_EH_;
    
    transformation_estimates_.push_back(new_estimate);
  }
    
  return true;
}


void TransformationEstimator::addLastRetrievedPosePair()
{  
  ROS_INFO("Saving last published poses for hand and eye...");
  
  int trialCount = 0;
  do
  {
    ros::spinOnce();
    
    if( !hand_recorded_ )
    {
      ROS_INFO("No hand pose retrieved yet. Waiting...");
    }
    if( !eye_recorded_ )
    {
      ROS_INFO("No eye pose retrieved yet. Waiting...");
    }
    if( trialCount++>=3 ) break;
    else if( !hand_recorded_ || !eye_recorded_ ) ROS_INFO("%i attempts remaining",3-trialCount);
    
  }while( ros_node_->ok() && ( !hand_recorded_ || !eye_recorded_ ) );
  
  if( hand_recorded_ && eye_recorded_ )
  {
    ros::Time currentTime = ros::Time::now();
    
    if( (currentTime - recorded_hand_time_stamp_) >= ros::Duration(5.0) )
    {
      ROS_WARN("Adding hand pose that has been recorded %f seconds ago.", (currentTime-recorded_hand_time_stamp_).toSec() );
    }
    if( (currentTime - recorded_eye_time_stamp_) >= ros::Duration(5.0) )
    {
      ROS_WARN("Adding eye pose that has been recorded %f seconds ago.", (currentTime-recorded_hand_time_stamp_).toSec() );
    }
    
    pose_pairs_.push_back( pair<geometry_msgs::Pose, geometry_msgs::Pose>(buffered_hand_,buffered_eye_) );
    
    calib_pattern_image_coordinates_.push_back( std::vector<hand_eye_calibration::Point2D>() );
    calibration_images_.push_back( cv::Mat() );
    
    if( pose_pairs_.size()>=2 ) //estimation is possible
    {
      calculateTransformation(true);
      
      EstimationData new_estimate;
      new_estimate.rot_EH = rot_EH_;
      new_estimate.E_trans_EH = E_trans_EH_;
      
      transformation_estimates_.push_back(new_estimate);
    }
  }
  
  
  return;
}


void TransformationEstimator::deleteLastAddedPosePair()
{
  if( pose_pairs_.size()>= 1)
  {
    del_pose_pairs_.push_back( pose_pairs_.back() );
    pose_pairs_.pop_back();
  }
  if( transformation_estimates_.size()>=1 )
  {
    transformation_estimates_.pop_back();
  }
  if( calib_pattern_image_coordinates_.size()>= 1 )
  {
    del_calib_pattern_image_coordinates_.push_back( calib_pattern_image_coordinates_.back() );
    calib_pattern_image_coordinates_.pop_back();
  }
  if( calibration_images_.size()>=1 )
  {
    del_calibration_images_.push_back( calibration_images_.back() );
    calibration_images_.pop_back();
  }
  
  return;
}


void TransformationEstimator::restoreLastDeletedPosePair()
{
  if( del_pose_pairs_.size()>= 1)
  {
    pose_pairs_.push_back( del_pose_pairs_.back() );
    del_pose_pairs_.pop_back();
  }
  if( del_calib_pattern_image_coordinates_.size()>= 1 )
  {
    calib_pattern_image_coordinates_.push_back( del_calib_pattern_image_coordinates_.back() );
    del_calib_pattern_image_coordinates_.pop_back();
  }
  if( del_calibration_images_.size()>=1 )
  {
    calibration_images_.push_back( del_calibration_images_.back() );
    del_calibration_images_.pop_back();
  }
  
  if( pose_pairs_.size()>=2 ) //estimation is possible
  {
    calculateTransformation(true);
    
    EstimationData new_estimate;
    new_estimate.rot_EH = rot_EH_;
    new_estimate.E_trans_EH = E_trans_EH_;
    
    transformation_estimates_.push_back(new_estimate);
  }
  return;
}

double TransformationEstimator::getReprojectionError()
{
  
}


geometry_msgs::Pose TransformationEstimator::getHandToEye()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::getTransformation() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  
  geometry_msgs::Pose estimatedTransformation;
  
  estimatedTransformation.position.x = E_trans_EH_[0];
  estimatedTransformation.position.y = E_trans_EH_[1];
  estimatedTransformation.position.z = E_trans_EH_[2];
  estimatedTransformation.orientation.x = rot_EH_.x();
  estimatedTransformation.orientation.y = rot_EH_.y();
  estimatedTransformation.orientation.z = rot_EH_.z();
  estimatedTransformation.orientation.w = rot_EH_.w();
  
  return estimatedTransformation;
}


Matrix3d TransformationEstimator::rotH2E()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::rotH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return rot_EH_.toRotationMatrix();
}


Matrix3d TransformationEstimator::rotE2H()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::rotE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return rot_EH_.inverse().toRotationMatrix();
}


Vector3d TransformationEstimator::transH2E()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::transH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return E_trans_EH_;
}


Vector3d TransformationEstimator::transE2H()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::transE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return -(rotH2E()*E_trans_EH_);
}


Matrix<double,4,4> TransformationEstimator::matrixH2E()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::matrixH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  Matrix<double,4,4> mH2E;
  
  Vector3d E_t_EH = E_trans_EH_;
  
  mH2E<<rotH2E(), E_t_EH, 0, 0, 0, 1;
  
  return mH2E;
}


Matrix<double,4,4> TransformationEstimator::matrixE2H()
{
  if( !transformation_calculated_ ) ROS_WARN("TransformationEstimator::matrixE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  Matrix<double,4,4> mE2H;
  
  Vector3d H_t_HE = - rotE2H()*E_trans_EH_;
  
  mE2H<<rotE2H(), H_t_HE, 0, 0, 0, 1;
  
  return mE2H;
}


void TransformationEstimator::clearAll()
{
  ROS_INFO("Deleting all buffered hand-eye pose pairs and associated data.");
  
  for( unsigned int i = 0; i<pose_pairs_.size(); i++ )
  {
    del_pose_pairs_.push_back( pose_pairs_[i] );
  }
  pose_pairs_.clear();
  
  for( unsigned int i = 0; i<calib_pattern_image_coordinates_.size(); i++ )
  {
    del_calib_pattern_image_coordinates_.push_back( calib_pattern_image_coordinates_[i] );
  }
  calib_pattern_image_coordinates_.clear();
  
  for( unsigned int i = 0; i<calibration_images_.size(); i++ )
  {
    del_calibration_images_.push_back( calibration_images_[i] );
  }
  calibration_images_.clear();
    
  transformation_estimates_.clear();
  
  
  ROS_INFO_STREAM("Number of pose pairs now available for computation: "<<pose_pairs_.size() );
  
  return;
}


void TransformationEstimator::setNewServiceWaitTime( unsigned int _wait_time )
{
  max_service_wait_time_.fromNSec( _wait_time * 1000000 );
  return;
}


void TransformationEstimator::startListening()
{
  hand_subscriber_ = ros_node_->subscribe("/hec/hand_position",1,&TransformationEstimator::handListening,this);
  eye_subscriber_ = ros_node_->subscribe("/hec/eye_position",1,&TransformationEstimator::eyeListening,this);
}
    

void TransformationEstimator::stopListening()
{
  hand_subscriber_.shutdown();
  eye_subscriber_.shutdown();
}


void TransformationEstimator::handListening( const geometry_msgs::PoseConstPtr& _newPose )
{  
  buffered_hand_ = *_newPose;
  hand_recorded_ = true;
  recorded_hand_time_stamp_ = ros::Time::now();
  return;
}


void TransformationEstimator::eyeListening( const geometry_msgs::PoseConstPtr& _newPose )
{  
  buffered_eye_ = *_newPose;
  eye_recorded_ = true;
  recorded_eye_time_stamp_ = ros::Time::now();
  return;
}


int TransformationEstimator::count()
{
  return pose_pairs_.size();
}


void TransformationEstimator::dumpTrash()
{
  del_pose_pairs_.clear();
  del_calib_pattern_image_coordinates_.clear();
  del_calibration_images_.clear();
}



bool TransformationEstimator::roots( double _aCoeff, double _bCoeff, double _cCoeff, pair<double,double>& _roots )
{
  double discriminant = sqrt( _bCoeff*_bCoeff - 4*_aCoeff*_cCoeff );
  
  if( discriminant<0 ) return false;
  
  _roots.first = (-_bCoeff + discriminant ) / (2*_aCoeff);
  _roots.second = (-_bCoeff - discriminant ) / (2*_aCoeff);
  return true;
}



bool TransformationEstimator::printToFile( string fileName_ )
{
  // create Mat arrays with the poses of hand and eye_position
  
  try
  {
    cv::Mat hand_poses(7,pose_pairs_.size() ,CV_64FC1);
    cv::Mat eye_poses(7,pose_pairs_.size() ,CV_64FC1);
        
    
    for( int i=0; i<pose_pairs_.size(); i++ )
    {
      hand_poses.at<double>(0,i) = pose_pairs_[i].first.orientation.x;
      hand_poses.at<double>(1,i) = pose_pairs_[i].first.orientation.y;
      hand_poses.at<double>(2,i) = pose_pairs_[i].first.orientation.z;
      hand_poses.at<double>(3,i) = pose_pairs_[i].first.orientation.w;
      hand_poses.at<double>(4,i) = pose_pairs_[i].first.position.x;
      hand_poses.at<double>(5,i) = pose_pairs_[i].first.position.y;
      hand_poses.at<double>(6,i) = pose_pairs_[i].first.position.z;
      
      eye_poses.at<double>(0,i) = pose_pairs_[i].second.orientation.x;
      eye_poses.at<double>(1,i) = pose_pairs_[i].second.orientation.y;
      eye_poses.at<double>(2,i) = pose_pairs_[i].second.orientation.z;
      eye_poses.at<double>(3,i) = pose_pairs_[i].second.orientation.w;
      eye_poses.at<double>(4,i) = pose_pairs_[i].second.position.x;
      eye_poses.at<double>(5,i) = pose_pairs_[i].second.position.y;
      eye_poses.at<double>(6,i) = pose_pairs_[i].second.position.z;
    }
    
    cv::Mat estimated_transformations( 10, transformation_estimates_.size(), CV_64FC1 );
    for( int i=0; i<transformation_estimates_.size(); i++ )
    {
      estimated_transformations.at<double>(0,i) = transformation_estimates_[i].rot_EH.x();
      estimated_transformations.at<double>(1,i) = transformation_estimates_[i].rot_EH.y();
      estimated_transformations.at<double>(2,i) = transformation_estimates_[i].rot_EH.z();
      estimated_transformations.at<double>(3,i) = transformation_estimates_[i].rot_EH.w();
      estimated_transformations.at<double>(4,i) = transformation_estimates_[i].E_trans_EH.x();
      estimated_transformations.at<double>(5,i) = transformation_estimates_[i].E_trans_EH.y();
      estimated_transformations.at<double>(6,i) = transformation_estimates_[i].E_trans_EH.z();
      estimated_transformations.at<double>(7,i) = transformation_estimates_[i].reprojection_error;
      estimated_transformations.at<double>(8,i) = transformation_estimates_[i].euler_rms_error;
      estimated_transformations.at<double>(9,i) = transformation_estimates_[i].relative_translation_error;
    }
    
    unsigned int pattern_coordinate_size=0;
    for( int i=0; i< calib_pattern_image_coordinates_.size(); i++ )
    {
      if( calib_pattern_image_coordinates_[i].size()!=0 )
      {
	pattern_coordinate_size = calib_pattern_image_coordinates_[i].size();
	break;
      }
    }
    cv::Mat pattern_placeholder(pattern_coordinate_size,1,CV_64FC1,-1000);
    
    // saving each pattern_coordinates set in two rows, the first for the x, the second for the y values, -1000 is a placeholder for unknown coordinates (since they're always in the image, they're always positive if known)
    cv::Mat pattern_coordinates( pattern_coordinate_size, 0, CV_64FC1);
    for( unsigned int i=0; i<calib_pattern_image_coordinates_.size(); i++ )
    {
      if( calib_pattern_image_coordinates_[i].size()!=0 )
      {
	cv::Mat x_row( pattern_coordinate_size, 1, CV_64FC1);
	cv::Mat y_row( pattern_coordinate_size, 1, CV_64FC1);
	
	for( int j=0; j<calib_pattern_image_coordinates_[i].size(); j++ )
	{
	  x_row.at<double>(j,0) = calib_pattern_image_coordinates_[i][j].x;
	  y_row.at<double>(j,0) = calib_pattern_image_coordinates_[i][j].y;
	}
	pattern_coordinates.push_back(x_row);
	pattern_coordinates.push_back(y_row);
      }
      else
      {
	pattern_coordinates.push_back( pattern_placeholder );
	pattern_coordinates.push_back( pattern_placeholder );
      }
    }
    
    cv::FileStorage outputFile( fileName_, cv::FileStorage::WRITE );
    
    outputFile<<"hand_poses"<<hand_poses;
    outputFile<<"eye_poses"<<eye_poses;
    outputFile<<"estimated_transformations"<<estimated_transformations;
    outputFile<<"pattern_coordinates"<<pattern_coordinates;
  }
  catch(...)
  {
    ROS_ERROR("TransformationEstimator::printToFile::failed.");
    return 0;
  }
  
  return 1;
}



bool TransformationEstimator::loadFromFile( string fileName_, bool destroyOldData_ )
{
  try
  {
    cv::FileStorage inputFile( fileName_, cv::FileStorage::READ );
    
    cv::Mat hand_poses( cv::Size(), CV_64FC1 );
    cv::Mat eye_poses( cv::Size(), CV_64FC1 );
    cv::Mat estimated_transformations( cv::Size(), CV_64FC1 );
    cv::Mat pattern_coordinates( cv::Size(), CV_64FC1 );
    
    inputFile["hand_poses"] >> hand_poses;
    inputFile["eye_poses"] >> eye_poses;
    inputFile["estimated_transformations"] >> estimated_transformations;
    inputFile["pattern_coordinates"] >> pattern_coordinates;
    
    if( hand_poses.cols!=eye_poses.cols || hand_poses.rows!=7 || eye_poses.rows!=7 || hand_poses.cols==0 || estimated_transformations.rows!=10 || estimated_transformations.cols!=(hand_poses.cols-1) || (pattern_coordinates.rows%2)!=0 )
    {
       ROS_ERROR("TransformationEstimator::loadFromFile::failed::The input file %s did not contain valid cv::Mat matrices.",fileName_.c_str() );
       return 0;
    }
    
    if( destroyOldData_ )
    {
      pose_pairs_.clear();
      transformation_estimates_.clear();
      del_pose_pairs_.clear();
      del_calib_pattern_image_coordinates_.clear();
      del_calibration_images_.clear();
    }
    
    for( int i=0; i<hand_poses.cols; i++ )
    {
     geometry_msgs::Pose handPoseToAdd, eyePoseToAdd;
     
     handPoseToAdd.orientation.x = hand_poses.at<double>(0,i);
     handPoseToAdd.orientation.y = hand_poses.at<double>(1,i);
     handPoseToAdd.orientation.z = hand_poses.at<double>(2,i);
     handPoseToAdd.orientation.w = hand_poses.at<double>(3,i);
     handPoseToAdd.position.x = hand_poses.at<double>(4,i);
     handPoseToAdd.position.y = hand_poses.at<double>(5,i);
     handPoseToAdd.position.z = hand_poses.at<double>(6,i);
     
     eyePoseToAdd.orientation.x = eye_poses.at<double>(0,i);
     eyePoseToAdd.orientation.y = eye_poses.at<double>(1,i);
     eyePoseToAdd.orientation.z = eye_poses.at<double>(2,i);
     eyePoseToAdd.orientation.w = eye_poses.at<double>(3,i);
     eyePoseToAdd.position.x = eye_poses.at<double>(4,i);
     eyePoseToAdd.position.y = eye_poses.at<double>(5,i);
     eyePoseToAdd.position.z = eye_poses.at<double>(6,i);
      
      pose_pairs_.push_back( pair<geometry_msgs::Pose, geometry_msgs::Pose>( handPoseToAdd,eyePoseToAdd ) );
    }
    for( int i=0; i<estimated_transformations.cols; i++ )
    {
      EstimationData new_estimate;
      
      new_estimate.rot_EH.x() = estimated_transformations.at<double>(0,i);
      new_estimate.rot_EH.y() = estimated_transformations.at<double>(1,i);
      new_estimate.rot_EH.z() = estimated_transformations.at<double>(2,i);
      new_estimate.rot_EH.w() = estimated_transformations.at<double>(3,i);
      
      new_estimate.E_trans_EH.x() = estimated_transformations.at<double>(4,i);
      new_estimate.E_trans_EH.y() = estimated_transformations.at<double>(5,i);
      new_estimate.E_trans_EH.z() = estimated_transformations.at<double>(6,i);
      
      new_estimate.reprojection_error = estimated_transformations.at<double>(7,i);
      new_estimate.euler_rms_error = estimated_transformations.at<double>(8,i);
      new_estimate.relative_translation_error = estimated_transformations.at<double>(9,i);
      
      transformation_estimates_.push_back(new_estimate);
      
    }
    for( int i=0; i<pattern_coordinates.rows; i+=2 )
    {
      if( pattern_coordinates.at<double>(i,0)==-1000 ) //placeholder for unknown data
      {
	calib_pattern_image_coordinates_.push_back( std::vector<hand_eye_calibration::Point2D>() );
      }
      else
      {
	std::vector<hand_eye_calibration::Point2D> new_pattern_coordinates;
	for( int j=0; j<pattern_coordinates.cols; j++ )
	{
	  hand_eye_calibration::Point2D new_point;
	  new_point.x = pattern_coordinates.at<double>(i,j);
	  new_point.y = pattern_coordinates.at<double>(i+1,j);
	  new_pattern_coordinates.push_back(new_point);
	}
	calib_pattern_image_coordinates_.push_back(new_pattern_coordinates);
      }
    }
    
  }
  catch(...)
  {
    ROS_ERROR("TransformationEstimator::loadFromFile::failed.");
    return 0;
  }
  return 1;
}



Matrix3d TransformationEstimator::crossProdMatrix( Vector3d _vec )
{
  Matrix3d ret;
  ret<< 0,		-_vec.z(),	_vec.y(),
	_vec.z(),	0,		-_vec.x(),
	-_vec.y(),	_vec.x(),	0;
  return ret;
}


Eigen::Vector3d TransformationEstimator::geometryToEigen( const geometry_msgs::Point& _vec )
{
  Eigen::Vector3d output;
  output.x() = _vec.x;
  output.y() = _vec.y;
  output.z() = _vec.z;
  return output;
}


Eigen::Quaterniond TransformationEstimator::geometryToEigen( const geometry_msgs::Quaternion& _quat )
{
  Eigen::Quaterniond output;
  output.x() = _quat.x;
  output.y() = _quat.y;
  output.z() = _quat.z;
  output.w() = _quat.w;
  return output;
}