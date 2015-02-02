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

CalibrationSetup::CalibrationSetup():
  is_setup_(false)
{
  
}

void CalibrationSetup::setData( Eigen::Matrix<double,3,4>& _camera_projection_matrix, std::vector<geometry_msgs::Point>& _calibration_pattern_world_coordinates )
{
  camera_projection_matrix_=_camera_projection_matrix;
  calibration_pattern_world_coordinates_=_calibration_pattern_world_coordinates;
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

geometry_msgs::Point CalibrationSetup::calibPointWorldCoordinates( unsigned int _i )
{
  return calibration_pattern_world_coordinates_[_i];
}

hand_eye_calibration::Point2D CalibrationSetup::getProjectedPointCoordinates( unsigned int _i, geometry_msgs::Pose _camera_pose )
{
  return hand_eye_calibration::Point2D(); //to do .////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

std::vector<hand_eye_calibration::Point2D> CalibrationSetup::getProjectedCoordinates( geometry_msgs::Pose _camera_pose )
{
  return std::vector<hand_eye_calibration::Point2D>(); //to do .////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}