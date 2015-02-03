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
#include <sstream>
#include <boost/foreach.hpp>
#include "utils/ros_eigen.h"

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
  
  PoseData new_data;
  
  // load camera pose information
  new_data.eye_pose = cam_pose_request.response.description.pose;
  
  sensor_msgs::Image cam_image = cam_pose_request.response.description.image;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_cam_image = (*cv_ptr).image;
    new_data.eye_image = cv_cam_image;
  }
  catch( cv_bridge::Exception& e )
  {
    ROS_WARN_STREAM("TransformationEstimator::addNewPosePair()::cv_bridge exception: "<<e.what()<<". No image is added." );
  }
  new_data.calibration_pattern_coordinates = cam_pose_request.response.description.point_coordinates;
 
  // load hand pose information
  new_data.hand_pose = hand_pose_request.response.description.pose;
  
  
  pose_data_.push_back( new_data );
  
    
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
    
    PoseData new_data;
    new_data.hand_pose = buffered_hand_;
    new_data.eye_pose = buffered_eye_;
    
  }
  
  
  return;
}


void TransformationEstimator::deleteLastAddedPosePair()
{
  if( pose_data_.size()>= 1)
  {
    del_pose_data_.push_back( pose_data_.back() );
    pose_data_.pop_back();
  }
  
  return;
}


void TransformationEstimator::restoreLastDeletedPosePair()
{
  if( del_pose_data_.size()>= 1)
  {
    pose_data_.push_back( del_pose_data_.back() );
    del_pose_data_.pop_back();
  }
  
  return;
}


bool TransformationEstimator::estimationPossible()
{
  return pose_data_.size()>=2;
}


void TransformationEstimator::createAndAddNewEstimation()
{
  transformation_estimates_.push_back( getNewEstimation() );
  return;
}

void TransformationEstimator::clearEstimations()
{
  transformation_estimates_.clear();
}

void TransformationEstimator::deleteLastEstimation()
{
  transformation_estimates_.pop_back();
}

TransformationEstimator::EstimationData TransformationEstimator::getNewEstimation()
{
  if( !estimationPossible() ) //estimation is NOT possible
  {
    std::string error_message = "TransformationEstimator::getNewEstimation:: estimation is not possible since not enough data is available.";
    ROS_ERROR_STREAM( error_message );
    std::runtime_error e( error_message );
    throw e;
  }
    
  calculateTransformation(true);
  
  EstimationData new_estimate;
  new_estimate.rot_EH = rot_EH_;
  new_estimate.E_trans_EH = E_trans_EH_;
  
  st_is::StdError reprojection_error;
  if( getReprojectionError(new_estimate,reprojection_error) )
  {
    new_estimate.setReprojectionError(reprojection_error);
  }
  
  std::pair<st_is::StdError,st_is::StdError> transformation_error = getTransformationError(new_estimate);
  new_estimate.setTransformationErrors( transformation_error );
  
  return new_estimate;
}

bool TransformationEstimator::getReprojectionError( EstimationData const& _estimation_data, st_is::StdError& _reprojection_error )
{
  if( !calibration_configuration_.isSetup() )
  {
    ROS_WARN("TransformationEstimator::getReprojectionError::Called but cannot calculate reprojection error since calibration setup configuration hasn't been setup, no camera and/or calibration pattern world coordinates are given.");
    return false;
  }
  if( pose_data_.size()<2 )
  {
    ROS_WARN("TransformationEstimator::getReprojectionError::Called but cannot calculate reprojection error because not enough data is available (number of pose measurements is insufficient).");
    return false;
  }
  unsigned int available_data_count = 0;
  std::vector<double> errors;
  
  for( unsigned int i=0; i<pose_data_.size(); i++ )
  {
    if( !pose_data_[i].calibration_pattern_coordinates.empty() )
    {
      errors.push_back( reprojectionErrorForTime(i,_estimation_data) );
      available_data_count++;
    }
  }
  
  if( available_data_count==0 ) return false; // no measured calibration pattern image coordinates were available
  
  _reprojection_error = st_is::StdError( errors ); // create error statistics
  
  return true;
}

double TransformationEstimator::reprojectionErrorForTime( unsigned int _i, EstimationData const& _estimation_data )
{
  if( pose_data_.size()>=_i )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorAtTimeAndPoint()::Given time index "<<_i<<" exceeds range of data container 'pose_data_' which has a size of "<<pose_data_.size()<<".";
    ROS_ERROR_STREAM( error.str() );
    throw std::range_error( error.str() );
  }
  else if( pose_data_[_i].calibration_pattern_coordinates.empty() )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorAtTimeAndPoint()::Given time index "<<_i<<" refers to a pose data set that features no data with the coordinates of the detected calibration pattern.";
    ROS_ERROR_STREAM( error.str() );
    throw std::invalid_argument( error.str() );
  }
  
  double accumulated_error = 0;
  unsigned int number_of_pattern_points = pose_data_[_i].calibration_pattern_coordinates.size();
  
  for( unsigned int j=0; j<number_of_pattern_points; j++ )
  {
    accumulated_error += reprojectionErrorForTimeAndPoint( _i, j, _estimation_data );
  }
  
  return accumulated_error/number_of_pattern_points;
}

double TransformationEstimator::reprojectionErrorForTimeAndPoint( unsigned int _i, unsigned int _j, EstimationData const& _estimation_data )
{
  if( pose_data_[_i].calibration_pattern_coordinates.size()>=_j )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorAtTimeAndPoint()::Given point index "<<_j<<" exceeds the size of the pattern coordinate container which has a size of "<<pose_data_[_i].calibration_pattern_coordinates.size()<<".";
    ROS_ERROR_STREAM( error.str() );
    throw std::range_error( error.str() );
  }
  double accumulated_error = 0;
  double number_of_poses = pose_data_.size();
  
  st_is::CoordinateTransformation t_EH( _estimation_data.rot_EH, _estimation_data.E_trans_EH ); // hand eye transformation estimate
  st_is::CoordinateTransformation t_HE = t_EH.inv();
  st_is::CoordinateTransformation t_BH_i( st_is::geometryToEigen(pose_data_[_i].hand_pose.orientation), st_is::geometryToEigen(pose_data_[_i].hand_pose.position) ); // hand to base at time i
  
  st_is::CoordinateTransformation t_BE_i = t_BH_i*t_HE;
  
  for( unsigned int k=0; k<number_of_poses; k++ )
  {
    if( k!=_i )
    {      
      st_is::CoordinateTransformation t_OE_k; // eye to calibration pattern coordinates at time k
      t_OE_k = st_is::CoordinateTransformation( st_is::geometryToEigen( pose_data_[k].eye_pose.orientation ),st_is::geometryToEigen( pose_data_[k].eye_pose.position ) ).inv();
      
      st_is::CoordinateTransformation t_HB_k; // base to hand at time k
      t_HB_k= st_is::CoordinateTransformation( st_is::geometryToEigen(pose_data_[k].hand_pose.orientation), st_is::geometryToEigen(pose_data_[k].hand_pose.position) ).inv();
      
      st_is::CoordinateTransformation t_0E_ik = t_OE_k*t_EH*t_HB_k*t_BE_i;
      
      geometry_msgs::Pose artificial_camera_pose;
      artificial_camera_pose.orientation = st_is::eigenToGeometry( t_0E_ik.rotation );
      artificial_camera_pose.position = st_is::eigenToGeometry( t_0E_ik.translation );
      
      accumulated_error += reprojectionErrorForTimeAndPointGivenProjection( _i, _j, artificial_camera_pose );
    }
  }
  return accumulated_error/(number_of_poses-1);
}

double TransformationEstimator::reprojectionErrorForTimeAndPointGivenProjection( unsigned int _i, unsigned int _j, geometry_msgs::Pose& _camera_pose )
{
  hand_eye_calibration::Point2D measured_coordinates = pose_data_[_i].calibration_pattern_coordinates[_j];
  hand_eye_calibration::Point2D projected_coordinates = calibration_configuration_.getProjectedPointCoordinates(_j,_camera_pose);
  
  double diff_x = measured_coordinates.x-projected_coordinates.x;
  double diff_y = measured_coordinates.y-projected_coordinates.y;
  
  double error = sqrt( diff_x*diff_x + diff_y*diff_y );
  
  return error;
}

std::pair<st_is::StdError,st_is::StdError> TransformationEstimator::getTransformationError( EstimationData const& _estimation_data )
{
  std::vector<double> absolut_euler_angle_errors; // euler angle: z,x,z
  std::vector<double> relative_translation_errors;
  
  BOOST_FOREACH( PoseData pose, pose_data_ )
  {
    st_is::CoordinateTransformation cam_pose_estimate = camPoseEstimateForPoseData( pose, _estimation_data );
    
    st_is::CoordinateTransformation cam_pose_measured;
    cam_pose_measured.rotation = st_is::geometryToEigen(pose.eye_pose.orientation);
    cam_pose_measured.translation = st_is::geometryToEigen(pose.eye_pose.position);
    
    Matrix<double,3,1> euler_estimated = cam_pose_estimate.rotation.matrix().eulerAngles(2, 0, 2);
    Matrix<double,3,1> euler_measured = cam_pose_measured.rotation.matrix().eulerAngles(2, 0, 2);
    Matrix<double,3,1> euler_difference = euler_estimated-euler_measured;
    
    double euler_error = sqrt( euler_difference(0,0)*euler_difference(0,0) + euler_difference(1,0)*euler_difference(1,0) + euler_difference(2,0)*euler_difference(2,0) );
    absolut_euler_angle_errors.push_back(euler_error);
    
    Eigen::Vector3d translation_difference = cam_pose_estimate.translation - cam_pose_measured.translation;
    double translation_diff_length = sqrt( translation_difference(0,0)*translation_difference(0,0) + translation_difference(1,0)*translation_difference(1,0) + translation_difference(2,0)*translation_difference(2,0) );
    double translation_measured_length = sqrt( cam_pose_measured.translation(0,0)*cam_pose_measured.translation(0,0) + cam_pose_measured.translation(1,0)*cam_pose_measured.translation(1,0) + cam_pose_measured.translation(2,0)*cam_pose_measured.translation(2,0) );
    
    double rel_translation_error = translation_diff_length/translation_measured_length;
    relative_translation_errors.push_back(rel_translation_error);
  }
  
  st_is::StdError euler_angle_error(absolut_euler_angle_errors);
  st_is::StdError relative_translation_error(relative_translation_errors);
  
  return std::pair<st_is::StdError,st_is::StdError>(euler_angle_error,relative_translation_error);
}

st_is::CoordinateTransformation TransformationEstimator::getCalibrationPatternPoseEstimate( EstimationData const& _estimation_data )
{
  if( pose_data_.empty() )
  {
    std::string msg = "TransformationEstimator::getCalibrationPatternPoseEstimate::Cannot calculate estimate since no pose data is available";
    ROS_ERROR_STREAM(msg);
    throw std::range_error(msg);
  }
  
  Eigen::Matrix<double,4,1> rotation_accumulator = Eigen::MatrixXd::Zero(4,1);
  Eigen::Vector3d translation_accumulator = Eigen::MatrixXd::Zero(3,1);
  
  st_is::CoordinateTransformation t_HE = st_is::CoordinateTransformation( _estimation_data.rot_EH, _estimation_data.E_trans_EH ).inv(); // inverse of hand eye transformation estimate
  
  BOOST_FOREACH( PoseData pose_data, pose_data_ )
  {
    st_is::CoordinateTransformation t_BH_i( st_is::geometryToEigen(pose_data.hand_pose.orientation), st_is::geometryToEigen(pose_data.hand_pose.position) );
    st_is::CoordinateTransformation t_EO_i( st_is::geometryToEigen(pose_data.eye_pose.orientation), st_is::geometryToEigen(pose_data.eye_pose.position) );
    
    st_is::CoordinateTransformation t_BO_i = t_BH_i*t_HE*t_EO_i;
    
    Eigen::Matrix<double,4,1> rot;
    int mult=1; // to ensure that only positive quaternions are used (since the mean is being calculated later)
    if( t_BO_i.rotation.w()<0 ) mult=-1;
    
    rot(0,0) = mult*t_BO_i.rotation.x();
    rot(1,0) = mult*t_BO_i.rotation.y();
    rot(2,0) = mult*t_BO_i.rotation.z();
    rot(3,0) = mult*t_BO_i.rotation.w();
    rotation_accumulator += rot;
    
    translation_accumulator += t_BO_i.translation;
  }
    
  Eigen::Quaterniond mean_rotation;
  mean_rotation.x() = rotation_accumulator(0,0)/pose_data_.size();
  mean_rotation.y() = rotation_accumulator(1,0)/pose_data_.size();
  mean_rotation.z() = rotation_accumulator(2,0)/pose_data_.size();
  mean_rotation.w() = rotation_accumulator(3,0)/pose_data_.size();
  
  Eigen::Vector3d mean_translation = translation_accumulator/pose_data_.size();
  
  return st_is::CoordinateTransformation( mean_rotation, mean_translation );
}

st_is::CoordinateTransformation TransformationEstimator::camPoseEstimateForPoseData( PoseData& _pose, EstimationData const& _estimation_data )
{
  st_is::CoordinateTransformation t_EH( _estimation_data.rot_EH, _estimation_data.E_trans_EH ); // inverse of hand eye transformation estimate
  st_is::CoordinateTransformation t_BO = getCalibrationPatternPoseEstimate( _estimation_data ); // robot base O, calibration pattern coordinate system O
  
  st_is::CoordinateTransformation t_HB_i = st_is::CoordinateTransformation( st_is::geometryToEigen(_pose.hand_pose.orientation), st_is::geometryToEigen(_pose.hand_pose.position) ).inv();
  
  return t_EH*t_HB_i*t_BO;
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
  
  for( unsigned int i = 0; i<pose_data_.size(); i++ )
  {
    del_pose_data_.push_back( pose_data_[i] );
  }
  pose_data_.clear();
    
  transformation_estimates_.clear();
  
  
  ROS_INFO_STREAM("Number of pose pairs now available for computation: "<<pose_data_.size() );
  
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
  return pose_data_.size();
}


void TransformationEstimator::dumpTrash()
{
  del_pose_data_.clear();
}



bool TransformationEstimator::printToFile( string fileName_ )
{
  // create Mat arrays with the poses of hand and eye_position
  
  try
  {
    
    // write pose data -------------------------------------
    cv::Mat hand_poses(7,pose_data_.size() ,CV_64FC1);
    cv::Mat eye_poses(7,pose_data_.size() ,CV_64FC1);
        
    
    for( int i=0; i<pose_data_.size(); i++ )
    {
      hand_poses.at<double>(0,i) = pose_data_[i].hand_pose.orientation.x;
      hand_poses.at<double>(1,i) = pose_data_[i].hand_pose.orientation.y;
      hand_poses.at<double>(2,i) = pose_data_[i].hand_pose.orientation.z;
      hand_poses.at<double>(3,i) = pose_data_[i].hand_pose.orientation.w;
      hand_poses.at<double>(4,i) = pose_data_[i].hand_pose.position.x;
      hand_poses.at<double>(5,i) = pose_data_[i].hand_pose.position.y;
      hand_poses.at<double>(6,i) = pose_data_[i].hand_pose.position.z;
      
      eye_poses.at<double>(0,i) = pose_data_[i].eye_pose.orientation.x;
      eye_poses.at<double>(1,i) = pose_data_[i].eye_pose.orientation.y;
      eye_poses.at<double>(2,i) = pose_data_[i].eye_pose.orientation.z;
      eye_poses.at<double>(3,i) = pose_data_[i].eye_pose.orientation.w;
      eye_poses.at<double>(4,i) = pose_data_[i].eye_pose.position.x;
      eye_poses.at<double>(5,i) = pose_data_[i].eye_pose.position.y;
      eye_poses.at<double>(6,i) = pose_data_[i].eye_pose.position.z;
    }
    
    unsigned int pattern_coordinate_size=0;
    for( int i=0; i< pose_data_.size(); i++ )
    {
      if( pose_data_[i].calibration_pattern_coordinates.size()!=0 )
      {
	pattern_coordinate_size = pose_data_[i].calibration_pattern_coordinates.size();
	break;
      }
    }
    cv::Mat pattern_placeholder(pattern_coordinate_size,1,CV_64FC1,-1000);
    
    // saving each pattern_coordinates set in two rows, the first for the x, the second for the y values, -1000 is a placeholder for unknown coordinates (since they're always in the image, they're always positive if known)
    cv::Mat pattern_coordinates( pattern_coordinate_size, 0, CV_64FC1);
    for( unsigned int i=0; i<pose_data_.size(); i++ )
    {
      if( pose_data_[i].calibration_pattern_coordinates.size()!=0 )
      {
	cv::Mat x_row( pattern_coordinate_size, 1, CV_64FC1);
	cv::Mat y_row( pattern_coordinate_size, 1, CV_64FC1);
	
	for( int j=0; j<pose_data_[i].calibration_pattern_coordinates.size(); j++ )
	{
	  x_row.at<double>(j,0) = pose_data_[i].calibration_pattern_coordinates[j].x;
	  y_row.at<double>(j,0) = pose_data_[i].calibration_pattern_coordinates[j].y;
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
    
    // write estimation data -----------------------------------------
    cv::Mat estimated_transformations( 13, transformation_estimates_.size(), CV_64FC1 );
    st_is::StdError reprojection_error, euler_angle_error, relative_translation_error;
    for( int i=0; i<transformation_estimates_.size(); i++ )
    {
      estimated_transformations.at<double>(0,i) = transformation_estimates_[i].rot_EH.x();
      estimated_transformations.at<double>(1,i) = transformation_estimates_[i].rot_EH.y();
      estimated_transformations.at<double>(2,i) = transformation_estimates_[i].rot_EH.z();
      estimated_transformations.at<double>(3,i) = transformation_estimates_[i].rot_EH.w();
      estimated_transformations.at<double>(4,i) = transformation_estimates_[i].E_trans_EH.x();
      estimated_transformations.at<double>(5,i) = transformation_estimates_[i].E_trans_EH.y();
      estimated_transformations.at<double>(6,i) = transformation_estimates_[i].E_trans_EH.z();
      if( transformation_estimates_[i].reprojectionError( reprojection_error ) )
      {
	estimated_transformations.at<double>(7,i) = reprojection_error.mean;
	estimated_transformations.at<double>(8,i) = sqrt(reprojection_error.variance);
      }
      else
      {
	estimated_transformations.at<double>(7,i) = -1000;
	estimated_transformations.at<double>(8,i) = -1000;
      }
      if( transformation_estimates_[i].eulerAngleError( euler_angle_error ) )
      {
	estimated_transformations.at<double>(9,i) = euler_angle_error.mean;
	estimated_transformations.at<double>(10,i) = sqrt(euler_angle_error.variance);
      }
      else
      {
	estimated_transformations.at<double>(9,i) = -1000;
	estimated_transformations.at<double>(10,i) = -1000;
      }
      if( transformation_estimates_[i].relativeTranslationError( relative_translation_error ) )
      {
	estimated_transformations.at<double>(11,i) = relative_translation_error.mean;
	estimated_transformations.at<double>(12,i) = sqrt(relative_translation_error.variance);
      }
      else
      {
	estimated_transformations.at<double>(11,i) = -1000;
	estimated_transformations.at<double>(12,i) = -1000;
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
    
    if( hand_poses.cols!=eye_poses.cols || hand_poses.rows!=7 || eye_poses.rows!=7 || hand_poses.cols==0 || estimated_transformations.rows!=10 || estimated_transformations.cols!=(hand_poses.cols-1) || (pattern_coordinates.rows!=2*hand_poses.cols)&&(pattern_coordinates.rows!=0) )
    {
       ROS_ERROR("TransformationEstimator::loadFromFile::failed::The input file %s did not contain valid cv::Mat matrices.",fileName_.c_str() );
       return 0;
    }
    
    if( destroyOldData_ )
    {
      pose_data_.clear();
      transformation_estimates_.clear();
      del_pose_data_.clear();
    }
    
    for( int i=0; i<hand_poses.cols; i++ )
    {
      PoseData new_data;
      geometry_msgs::Pose handPoseToAdd, eyePoseToAdd;
      
      new_data.hand_pose.orientation.x = hand_poses.at<double>(0,i);
      new_data.hand_pose.orientation.y = hand_poses.at<double>(1,i);
      new_data.hand_pose.orientation.z = hand_poses.at<double>(2,i);
      new_data.hand_pose.orientation.w = hand_poses.at<double>(3,i);
      new_data.hand_pose.position.x = hand_poses.at<double>(4,i);
      new_data.hand_pose.position.y = hand_poses.at<double>(5,i);
      new_data.hand_pose.position.z = hand_poses.at<double>(6,i);
      
      new_data.eye_pose.orientation.x = eye_poses.at<double>(0,i);
      new_data.eye_pose.orientation.y = eye_poses.at<double>(1,i);
      new_data.eye_pose.orientation.z = eye_poses.at<double>(2,i);
      new_data.eye_pose.orientation.w = eye_poses.at<double>(3,i);
      new_data.eye_pose.position.x = eye_poses.at<double>(4,i);
      new_data.eye_pose.position.y = eye_poses.at<double>(5,i);
      new_data.eye_pose.position.z = eye_poses.at<double>(6,i);
      
      if( pattern_coordinates.rows!=0 )
      {
	if( pattern_coordinates.at<double>(i*2,0)!=-1000 ) //placeholder for unknown data
	{
	  std::vector<hand_eye_calibration::Point2D> new_pattern_coordinates;
	  for( int j=0; j<pattern_coordinates.cols; j++ )
	  {
	    hand_eye_calibration::Point2D new_point;
	    new_point.x = pattern_coordinates.at<double>(i*2,j);
	    new_point.y = pattern_coordinates.at<double>(i*2+1,j);
	    new_data.calibration_pattern_coordinates.push_back(new_point);
	  }
	}
      }
      
      pose_data_.push_back(new_data);
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
      
      if( estimated_transformations.at<double>(7,i)!=-1000 )
      {
	st_is::StdError new_error;
	new_error.mean = estimated_transformations.at<double>(7,i);
	new_error.variance = estimated_transformations.at<double>(8,i)*estimated_transformations.at<double>(8,i);
	new_estimate.setReprojectionError( new_error );
      }
      if( estimated_transformations.at<double>(8,i)!=-1000 )
      {
	st_is::StdError new_angle_error;
	new_angle_error.mean = estimated_transformations.at<double>(9,i);
	new_angle_error.variance = estimated_transformations.at<double>(10,i)*estimated_transformations.at<double>(10,i);
	
	st_is::StdError new_translation_error;
	new_translation_error.mean = estimated_transformations.at<double>(11,i);
	new_translation_error.variance = estimated_transformations.at<double>(12,i)*estimated_transformations.at<double>(12,i);
	
	new_estimate.setTransformationErrors( new_angle_error, new_translation_error );
      }
      
      transformation_estimates_.push_back(new_estimate);
      
    }
    
  }
  catch(...)
  {
    ROS_ERROR("TransformationEstimator::loadFromFile::failed.");
    return 0;
  }
  return 1;
}


TransformationEstimator::EstimationData::EstimationData():
  has_reprojection_error_(false),
  has_transformation_error_(false)
{
  
}


void TransformationEstimator::EstimationData::setReprojectionError( st_is::StdError _error )
{
  has_reprojection_error_ = true;
  reprojection_error_ = _error;
}


void TransformationEstimator::EstimationData::setTransformationErrors( st_is::StdError _euler_angle_error, st_is::StdError _relative_translation_error )
{
  has_transformation_error_ = true;
  euler_angle_error_ = _euler_angle_error;
  relative_translation_error_ = _relative_translation_error;
}


void TransformationEstimator::EstimationData::setTransformationErrors( std::pair<st_is::StdError,st_is::StdError> _errors )
{
  setTransformationErrors(_errors.first,_errors.second);
}


bool TransformationEstimator::EstimationData::reprojectionError( st_is::StdError& _reprojection_error )
{
  if(!has_reprojection_error_) return false;
  
  _reprojection_error = reprojection_error_;
  return true;
}


bool TransformationEstimator::EstimationData::eulerAngleError( st_is::StdError& _euler_angle_error )
{
  if(!has_transformation_error_) return false;
  
  _euler_angle_error = euler_angle_error_;
  return true;
}

bool TransformationEstimator::EstimationData::relativeTranslationError( st_is::StdError& _relative_translation_error )
{
  if(!has_transformation_error_) return false;
  
  _relative_translation_error = relative_translation_error_;
  return true;
}