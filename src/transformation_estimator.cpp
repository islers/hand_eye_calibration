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
  //// to do neeeeeeeeeeeeext ///////////////////////////////////////////////////////////
  
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
    
    if( pose_pairs_.size()>=2 ) //estimation is possible
    {
      calculateTransformation(true);
      rotation_estimates_EH_.push_back( rot_EH_ );
      translation_estimates_E_t_EH_.push_back( E_trans_EH_ );
    }
  }
  
  
  return;
}



void TransformationEstimator::deleteLastAddedPosePair()
{
  pose_pairs_.pop_back();
  if( rotation_estimates_EH_.size()>=1 ) rotation_estimates_EH_.pop_back();
  if( translation_estimates_E_t_EH_.size()>=1 ) translation_estimates_E_t_EH_.pop_back();
  return;
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
  ROS_INFO("Deleting all buffered hand-eye pose pairs.");
  pose_pairs_.clear();
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
    cv::Mat handPoses(7,pose_pairs_.size() ,CV_64FC1);
    cv::Mat eyePoses(7,pose_pairs_.size() ,CV_64FC1);
    
    for( int i=0; i<pose_pairs_.size(); i++ )
    {
      handPoses.at<double>(0,i) = pose_pairs_[i].first.orientation.x;
      handPoses.at<double>(1,i) = pose_pairs_[i].first.orientation.y;
      handPoses.at<double>(2,i) = pose_pairs_[i].first.orientation.z;
      handPoses.at<double>(3,i) = pose_pairs_[i].first.orientation.w;
      handPoses.at<double>(4,i) = pose_pairs_[i].first.position.x;
      handPoses.at<double>(5,i) = pose_pairs_[i].first.position.y;
      handPoses.at<double>(6,i) = pose_pairs_[i].first.position.z;
      
      eyePoses.at<double>(0,i) = pose_pairs_[i].second.orientation.x;
      eyePoses.at<double>(1,i) = pose_pairs_[i].second.orientation.y;
      eyePoses.at<double>(2,i) = pose_pairs_[i].second.orientation.z;
      eyePoses.at<double>(3,i) = pose_pairs_[i].second.orientation.w;
      eyePoses.at<double>(4,i) = pose_pairs_[i].second.position.x;
      eyePoses.at<double>(5,i) = pose_pairs_[i].second.position.y;
      eyePoses.at<double>(6,i) = pose_pairs_[i].second.position.z;
    }
    
    cv::Mat estimatedTransformations( 7, rotation_estimates_EH_.size(), CV_64FC1 );
    for( int i=0; i<rotation_estimates_EH_.size(); i++ )
    {
      estimatedTransformations.at<double>(0,i) = rotation_estimates_EH_[i].x();
      estimatedTransformations.at<double>(1,i) = rotation_estimates_EH_[i].y();
      estimatedTransformations.at<double>(2,i) = rotation_estimates_EH_[i].z();
      estimatedTransformations.at<double>(3,i) = rotation_estimates_EH_[i].w();
      estimatedTransformations.at<double>(4,i) = translation_estimates_E_t_EH_[i].x();
      estimatedTransformations.at<double>(5,i) = translation_estimates_E_t_EH_[i].y();
      estimatedTransformations.at<double>(6,i) = translation_estimates_E_t_EH_[i].z();
    }
    
    cv::FileStorage outputFile( fileName_, cv::FileStorage::WRITE );
    
    outputFile<<"handPoses"<<handPoses;
    outputFile<<"eyePoses"<<eyePoses;
    outputFile<<"estimatedTransformations"<<estimatedTransformations;
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
    
    cv::Mat handPoses( cv::Size(), CV_64FC1 );
    cv::Mat eyePoses( cv::Size(), CV_64FC1 );
    cv::Mat estimatedTransformations( cv::Size(), CV_64FC1 );
    
    inputFile["handPoses"] >> handPoses;
    inputFile["eyePoses"] >> eyePoses;
    inputFile["estimatedTransformations"] >> estimatedTransformations;
    
    if( handPoses.cols!=eyePoses.cols || handPoses.rows!=7 || eyePoses.rows!=7 || handPoses.cols==0 || estimatedTransformations.rows!=7 || estimatedTransformations.cols!=(handPoses.cols-1) )
    {
       ROS_ERROR("TransformationEstimator::loadFromFile::failed::The input file %s did not contain valid cv::Mat matrices.",fileName_.c_str() );
       return 0;
    }
    
    if( destroyOldData_ )
    {
      pose_pairs_.clear();
      rotation_estimates_EH_.clear();
      translation_estimates_E_t_EH_.clear();
    }
    
    for( int i=0; i<handPoses.cols; i++ )
    {
     geometry_msgs::Pose handPoseToAdd, eyePoseToAdd;
     
     handPoseToAdd.orientation.x = handPoses.at<double>(0,i);
     handPoseToAdd.orientation.y = handPoses.at<double>(1,i);
     handPoseToAdd.orientation.z = handPoses.at<double>(2,i);
     handPoseToAdd.orientation.w = handPoses.at<double>(3,i);
     handPoseToAdd.position.x = handPoses.at<double>(4,i);
     handPoseToAdd.position.y = handPoses.at<double>(5,i);
     handPoseToAdd.position.z = handPoses.at<double>(6,i);
     
     eyePoseToAdd.orientation.x = eyePoses.at<double>(0,i);
     eyePoseToAdd.orientation.y = eyePoses.at<double>(1,i);
     eyePoseToAdd.orientation.z = eyePoses.at<double>(2,i);
     eyePoseToAdd.orientation.w = eyePoses.at<double>(3,i);
     eyePoseToAdd.position.x = eyePoses.at<double>(4,i);
     eyePoseToAdd.position.y = eyePoses.at<double>(5,i);
     eyePoseToAdd.position.z = eyePoses.at<double>(6,i);
      
      pose_pairs_.push_back( pair<geometry_msgs::Pose, geometry_msgs::Pose>( handPoseToAdd,eyePoseToAdd ) );
    }
    for( int i=0; i<estimatedTransformations.cols; i++ )
    {
      Quaterniond newRotationEstimate;
      newRotationEstimate.x() = estimatedTransformations.at<double>(0,i);
      newRotationEstimate.y() = estimatedTransformations.at<double>(1,i);
      newRotationEstimate.z() = estimatedTransformations.at<double>(2,i);
      newRotationEstimate.w() = estimatedTransformations.at<double>(3,i);
      Vector3d newTranslationEstimate;
      newTranslationEstimate.x() = estimatedTransformations.at<double>(4,i);
      newTranslationEstimate.y() = estimatedTransformations.at<double>(5,i);
      newTranslationEstimate.z() = estimatedTransformations.at<double>(6,i);
      
      rotation_estimates_EH_.push_back(newRotationEstimate);
      translation_estimates_E_t_EH_.push_back(newTranslationEstimate);
      
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