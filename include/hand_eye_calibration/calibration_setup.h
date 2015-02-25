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

#pragma once

#include "geometry_msgs/Pose.h"
#include "hand_eye_calibration/Point2D.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


/// class to store static information about the calibration setup (i.e. camera and calibration pattern) and to provide a few often used related methods (e.g. projecting world calibration pattern coordinates into image coordinates given the worlds pose relative to the camera frame)
/** @todo clean up redundant code
 */
class CalibrationSetup
{
public:
  CalibrationSetup();
  
  /// sets all needed data
  void setData( Eigen::Matrix<double,3,4>& _camera_projection_matrix, std::vector<geometry_msgs::Point>& _calibration_pattern_world_coordinates, unsigned int _image_height, unsigned int _image_width );
  
  /// returns true if the calibration setup is setup
  bool isSetup();
  
  /// returns the camera projection matrix
  Eigen::Matrix<double,3,4> cameraProjectionMatrix();
  
  /// returns the world coordinates of the calibration pattern points
  std::vector<geometry_msgs::Point> patternWorldCoordinates();
  
  // returns the image height;
  unsigned int imageHeight();
  
  /// returns the image width
  unsigned int imageWidth();
  
  /** Returns the calibration pattern coordinates in camera coordinate frame given the pose of the calibration pattern world relative to the camera
   * @param _camera_pose  The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO,  Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   * @param _pattern_coordinates Vector that gets filled with the coordinates (using push_back() ) - the vector is not emptied
   * @throws runtime_error if the object hasn't been setup yet
   */
  void getCameraFrameCoordinates( geometry_msgs::Pose _camera_pose, std::vector<geometry_msgs::Point>& _pattern_coordinates );
  
  /** Returns a vector with all projected coordinates of points given an array of the points in camera frame coordinates
   * @param _camera_coordinates Vector with the point coordinates in camera frame coordinates
   * @param _projected_coordinates vector with the points projected onto the camera image plane
   * @throws runtime_error if the object hasn't been setup yet
   */
  void getImageCoordinates( std::vector<geometry_msgs::Point>& _camera_coordinates, std::vector<hand_eye_calibration::Point2D>& _projected_coordinates );
  
  /// returns world coordinates of a calibration pattern point
  /**
   * @param _i index of the point
   */
  geometry_msgs::Point calibPointWorldCoordinates( unsigned int _i );
  
  /// returns the projected image coordinates for a point given the pose of the calibration pattern world relative to the camera
  /**
   * @param _i index of the point to be projected
   * @param _camera_pose The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO, Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   * @throws runtime_error if the object hasn't been setup yet
   * @throws range_error if _i is out of range
   */
  hand_eye_calibration::Point2D getProjectedPointCoordinates( unsigned int _i, geometry_msgs::Pose _camera_pose );
  
  /// returns a vector with all projected coordinates given the pose of the calibration pattern world relative to the camera
  /**
   * @param _camera_pose The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO,  Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   * @throws runtime_error if the object hasn't been setup yet
   */
  std::vector<hand_eye_calibration::Point2D> getProjectedCoordinates( geometry_msgs::Pose _camera_pose );
  
  /** returns the full projection matrix including the given pose and the camera matrix K
   * @param _camera_pose The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO,  Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
   * @throws runtime_error if the object hasn't been setup yet
   */
  Eigen::Matrix<double,3,4> fullProjectionMatrix( geometry_msgs::Pose _camera_pose );
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  unsigned int image_width_, image_height_;
  bool is_setup_;
  Eigen::Matrix<double,3,4> camera_projection_matrix_;
  std::vector<geometry_msgs::Point> calibration_pattern_world_coordinates_;
};