/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
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

#include "hand_eye_calibration/calibration_setup.h"
#include "utils/ros_eigen.h"

CalibrationSetup::CalibrationSetup():
  is_setup_(false)
{
  
}

void CalibrationSetup::setData( Eigen::Matrix<double,3,4>& _camera_projection_matrix, std::vector<geometry_msgs::Point>& _calibration_pattern_world_coordinates, unsigned int _image_height, unsigned int _image_width )
{
  camera_projection_matrix_=_camera_projection_matrix;
  calibration_pattern_world_coordinates_=_calibration_pattern_world_coordinates;
  image_height_ = _image_height;
  image_width_ = _image_width;
  is_setup_=true;
}

bool CalibrationSetup::isSetup()
{
  return is_setup_;
}

Eigen::Matrix<double,3,4> CalibrationSetup::cameraProjectionMatrix()
{
  return camera_projection_matrix_;
}

std::vector<geometry_msgs::Point> CalibrationSetup::patternWorldCoordinates()
{
  return calibration_pattern_world_coordinates_;
}

unsigned int CalibrationSetup::imageHeight()
{
  return image_height_;
}

unsigned int CalibrationSetup::imageWidth()
{
  return image_width_;
}

geometry_msgs::Point CalibrationSetup::calibPointWorldCoordinates( unsigned int _i )
{
  return calibration_pattern_world_coordinates_[_i];
}

hand_eye_calibration::Point2D CalibrationSetup::getProjectedPointCoordinates( unsigned int _i, geometry_msgs::Pose _camera_pose )
{
  if( !isSetup() )
  {
    std::runtime_error e("CalibrationSetup::getProjectedPointCoordinates::Can't be executed since the object hasn't been initialized yet.");
    throw e;
  }
  else if( calibration_pattern_world_coordinates_.size()<=_i )
  {
    std::stringstream msg;
    msg<<"CalibrationSetup::getProjectedPointCoordinates::Can't be executed since the given point index _i="<<_i<<" exceeds the range of the calibration pattern point container which is of size "<<calibration_pattern_world_coordinates_.size()<<".";
    std::range_error e( msg.str() );
    throw e;
  }
  
  // point coordinates in 3d world coordinates
  Eigen::Matrix<double,4,1> hom_calib_point_world_frame_3d;
  hom_calib_point_world_frame_3d(0,0) = calibration_pattern_world_coordinates_[_i].x;
  hom_calib_point_world_frame_3d(1,0) = calibration_pattern_world_coordinates_[_i].y;
  hom_calib_point_world_frame_3d(2,0) = calibration_pattern_world_coordinates_[_i].z;
  hom_calib_point_world_frame_3d(3,0) = 1;
  
  // project point
  Eigen::Matrix<double,3,4> full_projection_matrix = fullProjectionMatrix( _camera_pose );
  Eigen::Matrix<double,3,1> calib_point_image_frame_unnormalized;
  calib_point_image_frame_unnormalized = full_projection_matrix*hom_calib_point_world_frame_3d;
  
  // normalize (true perspective)
  hand_eye_calibration::Point2D calib_point_image_coordinates;
  calib_point_image_coordinates.x = calib_point_image_frame_unnormalized(0,0)/calib_point_image_frame_unnormalized(2,0);
  calib_point_image_coordinates.y = calib_point_image_frame_unnormalized(1,0)/calib_point_image_frame_unnormalized(2,0);
  
  return calib_point_image_coordinates;
}

std::vector<hand_eye_calibration::Point2D> CalibrationSetup::getProjectedCoordinates( geometry_msgs::Pose _camera_pose )
{
  if( !isSetup() )
  {
    std::runtime_error e("CalibrationSetup::getProjectedPointCoordinates::Can't be executed since the object hasn't been initialized yet.");
    throw e;
  }
  
  // build matrix with point coordinates
  Eigen::Matrix<double,4,Eigen::Dynamic> hom_calib_pattern_world_frame_3d;
  hom_calib_pattern_world_frame_3d.resize( 4,calibration_pattern_world_coordinates_.size() );
  
  for( unsigned int i=0; i<calibration_pattern_world_coordinates_.size(); i++ )
  {
    hom_calib_pattern_world_frame_3d(0,i) = calibration_pattern_world_coordinates_[i].x;
    hom_calib_pattern_world_frame_3d(1,i) = calibration_pattern_world_coordinates_[i].y;
    hom_calib_pattern_world_frame_3d(2,i) = calibration_pattern_world_coordinates_[i].z;
    hom_calib_pattern_world_frame_3d(3,i) = 1;
  }
  
  // project points
  Eigen::Matrix<double,3,4> full_projection_matrix = fullProjectionMatrix( _camera_pose );
  
  
  Eigen::Matrix<double,3,Eigen::Dynamic> calib_pattern_image_coordinates_unnormalized;
  calib_pattern_image_coordinates_unnormalized = full_projection_matrix*hom_calib_pattern_world_frame_3d;
  
  // normalize points (true perspective)
  std::vector<hand_eye_calibration::Point2D> calib_pattern_image_coordinates;
  
  for( unsigned int i=0; i<calibration_pattern_world_coordinates_.size(); i++ )
  {
    hand_eye_calibration::Point2D new_coordinates;
    new_coordinates.x = calib_pattern_image_coordinates_unnormalized(0,i)/calib_pattern_image_coordinates_unnormalized(2,i);
    new_coordinates.y = calib_pattern_image_coordinates_unnormalized(1,i)/calib_pattern_image_coordinates_unnormalized(2,i);
    calib_pattern_image_coordinates.push_back(new_coordinates);
  }
  
  return calib_pattern_image_coordinates;
}

Eigen::Matrix<double,3,4> CalibrationSetup::fullProjectionMatrix( geometry_msgs::Pose _camera_pose )
{
  Eigen::Matrix<double,4,4> projection_matrix = Eigen::MatrixXd::Zero(4,4);
  projection_matrix.topRows<3>()= st_is::transformationMatrix( _camera_pose );
  projection_matrix(3,3) = 1; // (homogenous coordinates)
  
  Eigen::Matrix<double,3,4> full_projection = camera_projection_matrix_*projection_matrix;
  return full_projection;
}