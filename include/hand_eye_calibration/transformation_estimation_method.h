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

#pragma once

#include "hand_eye_calibration/transformation_estimator.h"

/// interface class for transformation methods
class TransformationEstimator::EstimationMethod
{
public:
  class ErrorEvaluation;
  
  /** Returns the name of the method */
  virtual std::string estimationMethod()=0;
  
  /** calculates the transformation estimate
   * @param _pose_data data on which the estimation will be calculated
   * @param _suppressWarnings whether warnings are displayed or not
   * @throws std::runtime_error If the given pose_data is insufficient for hand-eye estimation
   * @return EstimationData without error estimates
   */
  virtual EstimationData calculateTransformation( std::vector<PoseData>& _pose_data, bool _suppressWarnings=false )=0;
  
  /** alias function to calculate the transformation estimate
   * @param _pose_data data on which the estimation will be calculated
   * @param _suppressWarnings whether warnings are displayed or not
   * @throws std::runtime_error If the given pose_data is insufficient for hand-eye estimation
   * @return EstimationData without error estimates
   */
  virtual EstimationData operator()( std::vector<PoseData>& _pose_data, bool _suppressWarnings=false )=0;
};