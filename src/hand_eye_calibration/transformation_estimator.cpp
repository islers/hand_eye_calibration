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

#include <sstream>
#include <boost/foreach.hpp>

#include "hand_eye_calibration/transformation_estimator.h" 
#include "hand_eye_calibration/transformation_estimation_method.h"
#include "hand_eye_calibration/estimation_data.h" 
#include "hand_eye_calibration/CameraPoseInfo.h"

#include "utils/ros_eigen.h"

using namespace std;
using namespace Eigen;

TransformationEstimator::TransformationEstimator( ros::NodeHandle* _n ):
max_service_wait_time_(2,100000000)
{
  ros_node_ = _n;
  
  
  hand_recorded_ = false;
  eye_recorded_ = false;
}


TransformationEstimator::~TransformationEstimator()
{
  
}


std::string TransformationEstimator::estimationMethod()
{
  if( estimation_methods_.empty() ) return "none";
  
  return estimation_methods_[0]->estimationMethod();
}


void TransformationEstimator::setEstimationMethod( boost::shared_ptr<EstimationMethod> _new_method )
{
  if( estimation_methods_.empty() ) estimation_methods_.push_back( _new_method );
  else estimation_methods_[0] = _new_method;
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

bool TransformationEstimator::addAndEstimate()
{
  bool return_value = addNewPosePair();
  bool estimation_possible = estimationPossible();
  return_value = return_value && estimation_possible;
  
  if( estimation_possible )
    createAndAddNewEstimation();
  
  return return_value;
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
    
    pose_data_.push_back( new_data );
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
  return pose_data_.size()>2 && !estimation_methods_.empty();
}

TransformationEstimator::EstimationData TransformationEstimator::estimate()
{
  if( transformation_estimates_.size()==0 )
  {
    return getNewEstimation();
  }
  else return transformation_estimates_.back();
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
    std::string error_message = "TransformationEstimator::getNewEstimation:: estimation is not possible since not enough data is available or no estimation method was specified.";
    ROS_ERROR_STREAM( error_message );
    std::runtime_error e( error_message );
    throw e;
  }
    
  EstimationData new_estimate = (*estimation_methods_[0])(pose_data_,true);
    
  st_is::StdError reprojection_error;
  if( getReprojectionError(new_estimate,reprojection_error) )
  {
    new_estimate.setReprojectionError(reprojection_error);
  }
  
  TransformationError transformation_error = getTransformationError(new_estimate);
  new_estimate.setTransformationErrors( transformation_error );
  
  return new_estimate;
}

bool TransformationEstimator::getReprojectionError( EstimationData& _estimation_data, st_is::StdError& _reprojection_error )
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

double TransformationEstimator::reprojectionErrorForTime( unsigned int _i, EstimationData& _estimation_data )
{
  if( pose_data_.size()<=_i )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorForTime()::Given time index "<<_i<<" exceeds range of data container 'pose_data_' which has a size of "<<pose_data_.size()<<".";
    ROS_ERROR_STREAM( error.str() );
    throw std::range_error( error.str() );
  }
  else if( pose_data_[_i].calibration_pattern_coordinates.empty() )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorForTime()::Given time index "<<_i<<" refers to a pose data set that features no data with the coordinates of the detected calibration pattern.";
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

double TransformationEstimator::reprojectionErrorForTimeAndPoint( unsigned int _i, unsigned int _j, EstimationData& _estimation_data )
{
  if( pose_data_[_i].calibration_pattern_coordinates.size()<=_j )
  {
    std::stringstream error;
    error << "TransformationEstimator::reprojectionErrorAtTimeAndPoint()::Given point index "<<_j<<" exceeds the size of the pattern coordinate container which has a size of "<<pose_data_[_i].calibration_pattern_coordinates.size()<<".";
    ROS_ERROR_STREAM( error.str() );
    throw std::range_error( error.str() );
  }
  double accumulated_error = 0;
  double number_of_poses = pose_data_.size();
  
  st_is::CoordinateTransformation t_EH( _estimation_data.rot_EH(), _estimation_data.E_trans_EH() ); // hand eye transformation estimate
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

TransformationEstimator::TransformationError TransformationEstimator::getTransformationError( EstimationData& _estimation_data )
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
  
  TransformationError transformation_error;
  transformation_error.euler_angle_error = st_is::StdError(absolut_euler_angle_errors);
  transformation_error.relative_translation_error = st_is::StdError(relative_translation_errors);
  
  return transformation_error;
}

st_is::CoordinateTransformation TransformationEstimator::lastHandPose()
{
  return st_is::CoordinateTransformation( st_is::geometryToEigen(pose_data_.back().hand_pose.orientation), st_is::geometryToEigen(pose_data_.back().hand_pose.position) );
}

st_is::CoordinateTransformation TransformationEstimator::lastEyePose()
{
  return st_is::CoordinateTransformation( st_is::geometryToEigen(pose_data_.back().eye_pose.orientation), st_is::geometryToEigen(pose_data_.back().eye_pose.position) );
}

st_is::CoordinateTransformation TransformationEstimator::getCalibrationPatternPoseEstimate( EstimationData& _estimation_data )
{
  if( pose_data_.empty() )
  {
    std::string msg = "TransformationEstimator::getCalibrationPatternPoseEstimate::Cannot calculate estimate since no pose data is available";
    ROS_ERROR_STREAM(msg);
    throw std::range_error(msg);
  }
  
  Eigen::Matrix<double,4,1> rotation_accumulator = Eigen::MatrixXd::Zero(4,1);
  Eigen::Vector3d translation_accumulator = Eigen::MatrixXd::Zero(3,1);
  
  st_is::CoordinateTransformation t_HE = st_is::CoordinateTransformation( _estimation_data.rot_EH(), _estimation_data.E_trans_EH() ).inv(); // inverse of hand eye transformation estimate
  
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

CalibrationSetup TransformationEstimator::getCalibrationSetup()
{
  return calibration_configuration_;
}

void TransformationEstimator::setCalibrationConfiguration( CalibrationSetup& _new_configuration )
{
  calibration_configuration_=_new_configuration;
}

void TransformationEstimator::setCalibrationConfiguration( Eigen::Matrix<double,3,4>& _projection_matrix, std::vector<geometry_msgs::Point>& _calibration_pattern_world_coordinates, unsigned int _image_height, unsigned int _image_width )
{
  calibration_configuration_.setData( _projection_matrix, _calibration_pattern_world_coordinates, _image_height, _image_width );
}

bool TransformationEstimator::loadCalibrationConfigurationFromService()
{
    
  hand_eye_calibration::CameraPoseInfo cam_pose_info;
  if( !ros::service::call("hec_eye_node_info",cam_pose_info) )
  {
    std::string warning = "TransformationEstimator::loadCalibrationConfigurationFromService::could not contact 'hec_eye_node_info'-service of cam pose publisher node. Thus no new information about the calibration setup is available.";
    ROS_WARN("%s",warning.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Successfully contacted 'hec_eye_node_info' service.");
    
    
    if( cam_pose_info.response.info.camera_info.P.size()!=12 )
    {
      ROS_ERROR("The projection matrix provided by the 'hec_eye_node_info' service is invalid or empty. Cannot setup calibration configuration.");
      return false;
    }
    if( cam_pose_info.response.info.pattern_coordinates.size()==0 )
    {
      ROS_ERROR("No calibraton pattern coordinates are provided by the 'hec_eye_nod_info' service. Cannot setup calibration configuration.");
      return false;
    }
    if( cam_pose_info.response.info.camera_info.height==0 || cam_pose_info.response.info.camera_info.width==0 )
    {
      ROS_ERROR_STREAM("The image height and/or width provided by the 'hec_eye_nod_info' service are invalid. The provided height is "<<cam_pose_info.response.info.camera_info.height<<"px, the provided width: "<<cam_pose_info.response.info.camera_info.width<<"px. Cannot setup calibration configuration.");
      return false;
    }
    unsigned int height = cam_pose_info.response.info.camera_info.height;
    unsigned int width = cam_pose_info.response.info.camera_info.width;
    
    Eigen::Matrix<double,3,4> projection_matrix;
    int i = 0;
    for( unsigned int row=0; row<3; row++ )
    {
      for( unsigned int col=0; col<4; col++, i++ )
      {
	projection_matrix(row,col) = cam_pose_info.response.info.camera_info.P[i];
      }
    }
    ROS_INFO_STREAM("The projection matrix is:"<<std::endl<<std::endl<<projection_matrix<<std::endl);
    
    std::vector<geometry_msgs::Point> calibration_pattern_world_coordinates = cam_pose_info.response.info.pattern_coordinates;
    
    ROS_INFO_STREAM("Calibration pattern retrieved with "<<calibration_pattern_world_coordinates.size()<<" points.");
    ROS_INFO_STREAM("Successfully retrieved all data needed from 'hec_eye_node_info' topic.");
    
    calibration_configuration_.setData( projection_matrix, calibration_pattern_world_coordinates, height, width );
    
    return true;
  }
}

st_is::CoordinateTransformation TransformationEstimator::camPoseEstimateForPoseData( PoseData& _pose, EstimationData& _estimation_data )
{
  st_is::CoordinateTransformation t_EH( _estimation_data.rot_EH(), _estimation_data.E_trans_EH() ); // inverse of hand eye transformation estimate
  st_is::CoordinateTransformation t_BO = getCalibrationPatternPoseEstimate( _estimation_data ); // robot base B, calibration pattern coordinate system O
  
  st_is::CoordinateTransformation t_HB_i = st_is::CoordinateTransformation( st_is::geometryToEigen(_pose.hand_pose.orientation), st_is::geometryToEigen(_pose.hand_pose.position) ).inv();
  
  return t_EH*t_HB_i*t_BO;
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
    cv::FileStorage outputFile( fileName_, cv::FileStorage::WRITE );
     
    
    // write calibration setup data
    outputFile<<"calibration_setup_is_setup"<<calibration_configuration_.isSetup();
    
    if( calibration_configuration_.isSetup() )
    {
      cv::Mat camera_projection_matrix_buff(3,4,CV_64FC1);
      Eigen::Matrix<double,3,4> camera_projection_matrix = calibration_configuration_.cameraProjectionMatrix();
      
      for( unsigned int row=0; row<3; row++ )
      {
	for( unsigned int col=0; col<4; col++ )
	{
	  camera_projection_matrix_buff.at<double>(row,col) = camera_projection_matrix(row,col);
	}
      }
      
      std::vector<geometry_msgs::Point> calibration_pattern_world_coordinates = calibration_configuration_.patternWorldCoordinates();
      cv::Mat calibration_pattern_world_coordinates_buff( 3,calibration_pattern_world_coordinates.size(),CV_64FC1 );
      
      for( unsigned int i=0; i<calibration_pattern_world_coordinates.size(); i++ )
      {
	calibration_pattern_world_coordinates_buff.at<double>(0,i) = calibration_pattern_world_coordinates[i].x;
	calibration_pattern_world_coordinates_buff.at<double>(1,i) = calibration_pattern_world_coordinates[i].y;
	calibration_pattern_world_coordinates_buff.at<double>(2,i) = calibration_pattern_world_coordinates[i].z;
      }
      
      outputFile<<"camera_projection_matrix"<<camera_projection_matrix_buff;
      outputFile<<"calibration_pattern_world_coordinates"<<calibration_pattern_world_coordinates_buff;
      outputFile<<"image_height"<<(int)calibration_configuration_.imageHeight();
      outputFile<<"image_width"<<(int)calibration_configuration_.imageWidth();
      
    }
    
    
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
	cv::Mat x_row( 1, pattern_coordinate_size, CV_64FC1);
	cv::Mat y_row( 1, pattern_coordinate_size, CV_64FC1);
	
	for( int j=0; j<pose_data_[i].calibration_pattern_coordinates.size(); j++ )
	{
	  x_row.at<double>(0,j) = pose_data_[i].calibration_pattern_coordinates[j].x;
	  y_row.at<double>(0,j) = pose_data_[i].calibration_pattern_coordinates[j].y;
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
    
    std::string estimation_method = estimation_methods_[0]->estimationMethod();
    
    cv::Mat estimated_transformations( 13, transformation_estimates_.size(), CV_64FC1 );
    st_is::StdError reprojection_error, euler_angle_error, relative_translation_error;
    
    for( int i=0; i<transformation_estimates_.size(); i++ )
    {
      estimated_transformations.at<double>(0,i) = transformation_estimates_[i].rot_EH().x();
      estimated_transformations.at<double>(1,i) = transformation_estimates_[i].rot_EH().y();
      estimated_transformations.at<double>(2,i) = transformation_estimates_[i].rot_EH().z();
      estimated_transformations.at<double>(3,i) = transformation_estimates_[i].rot_EH().w();
      estimated_transformations.at<double>(4,i) = transformation_estimates_[i].E_trans_EH().x();
      estimated_transformations.at<double>(5,i) = transformation_estimates_[i].E_trans_EH().y();
      estimated_transformations.at<double>(6,i) = transformation_estimates_[i].E_trans_EH().z();
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
    
    outputFile<<"hand_poses"<<hand_poses;
    outputFile<<"eye_poses"<<eye_poses;
    outputFile<<"estimation_method"<<estimation_method;
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
  using namespace std;
  
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
    
    if( hand_poses.cols!=eye_poses.cols || hand_poses.rows!=7 || eye_poses.rows!=7 || hand_poses.cols==0 || (estimated_transformations.rows!=13&&estimated_transformations.rows!=0) || (pattern_coordinates.cols!=2*hand_poses.cols)&&(pattern_coordinates.cols!=0) )
    {
       ROS_ERROR("TransformationEstimator::loadFromFile::failed::The input file %s did not contain valid cv::Mat matrices.",fileName_.c_str() );
       return 0;
    }
    
    if( destroyOldData_ )
    {
      pose_data_.clear();
      transformation_estimates_.clear();
      del_pose_data_.clear();
      
      // deletes old calibration setup data
      calibration_configuration_ = CalibrationSetup();
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
      
      
      /*// for bad data read
      for( int j=0;j<48;j++ )
      {
	hand_eye_calibration::Point2D new_point;
	new_point.x = pattern_coordinates.at<double>(0,i*96+j);
	new_point.y = pattern_coordinates.at<double>(0,i*96+j+48);
	new_data.calibration_pattern_coordinates.push_back(new_point);
      }*/
      
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
    
    
    std::string estimation_method;
    inputFile["estimation_method"] >> estimation_method;
    
    for( int i=0; i<estimated_transformations.cols; i++ )
    {
      
      Eigen::Quaterniond rot_EH;
      Eigen::Vector3d E_trans_EH;
      
      rot_EH.x() = estimated_transformations.at<double>(0,i);
      rot_EH.y() = estimated_transformations.at<double>(1,i);
      rot_EH.z() = estimated_transformations.at<double>(2,i);
      rot_EH.w() = estimated_transformations.at<double>(3,i);
      
      E_trans_EH.x() = estimated_transformations.at<double>(4,i);
      E_trans_EH.y() = estimated_transformations.at<double>(5,i);
      E_trans_EH.z() = estimated_transformations.at<double>(6,i);
      
      EstimationData new_estimate(estimation_method, rot_EH, E_trans_EH );
      
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
    
    
    // load calibration setup data if available
    bool calibration_was_setup=false;;
    inputFile["calibration_setup_is_setup"] >> calibration_was_setup;
    
    if( calibration_was_setup )
    {
      ROS_INFO("TransformationEstimator::loadFromFile::Calibration was setup. Loading camera projection matrix and calibration pattern world coordinates.");
      
      
      cv::Mat camera_projection_matrix_buff( cv::Size(), CV_64FC1 );
      cv::Mat calibration_pattern_world_coordinates_buff( cv::Size(), CV_64FC1 );
      
      inputFile["camera_projection_matrix"] >> camera_projection_matrix_buff;
      inputFile["calibration_pattern_world_coordinates"] >> calibration_pattern_world_coordinates_buff;
      
      int image_height, image_width;
      inputFile["image_height"] >> image_height;
      inputFile["image_width"] >> image_width;
      
      if( camera_projection_matrix_buff.rows!=3 || camera_projection_matrix_buff.cols!=4 || calibration_pattern_world_coordinates_buff.rows!=3 || image_height<=0 || image_width<=0 )
      {
	ROS_ERROR("TransformationEstimator::loadFromFile::the projection matrix or the coordinate matrix are ill formed or invalid image sizes are given. Cannot setup the calibration data. Check your data file.");
      }
      else
      {
	
	Eigen::Matrix<double,3,4> camera_projection_matrix;
	
	for( unsigned int row=0; row<3; row++ )
	{
	  for( unsigned int col=0; col<4; col++ )
	  {
	    camera_projection_matrix(row,col) = camera_projection_matrix_buff.at<double>(row,col);
	  }
	}
	
	std::vector<geometry_msgs::Point> calibration_pattern_world_coordinates;
	
	for( unsigned int i=0; i<calibration_pattern_world_coordinates_buff.cols; i++ )
	{
	  geometry_msgs::Point calibration_point;
	  calibration_point.x = calibration_pattern_world_coordinates_buff.at<double>(0,i);
	  calibration_point.y = calibration_pattern_world_coordinates_buff.at<double>(1,i);
	  calibration_point.z = calibration_pattern_world_coordinates_buff.at<double>(2,i);
	  calibration_pattern_world_coordinates.push_back( calibration_point );
	}

	calibration_configuration_.setData( camera_projection_matrix, calibration_pattern_world_coordinates, (unsigned int)image_height, (unsigned int)image_width );
	
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

void TransformationEstimator::addPose( PoseData& _new_pose )
{
  pose_data_.push_back(_new_pose );
}

std::vector<TransformationEstimator::PoseData> TransformationEstimator::poseData()
{
  return pose_data_;
}
