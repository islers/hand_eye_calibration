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

 
#include "hand_eye_calibration/autonomous_hand_eye_calibrator.h"

#include "utils/ros_eigen.h"
#include "hand_eye_calibration/estimation_data.h"
#include "hand_eye_calibration/pose_creator.h"
#include <random>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomous_hand_eye_calibration");
  ros::NodeHandle n("autonomous_hand_eye_calibration");
  
    
  AutonomousHandEyeCalibrator calibrator(&n);
  
  
  ros::Rate rate(0.2);
  while ( calibrator.runSingleIteration() && n.ok() )
  {
    cout<<endl<<"Number of added pose pairs: "<<calibrator.count();
    if( calibrator.estimateAvailable() )
    {
      TransformationEstimator::EstimationData estimate = calibrator.getEstimate();
      st_is::StdError reprojection_error;
      TransformationEstimator::TransformationError relative_error;
      
      cout<<endl<<"The current hand-eye-transformation estimate T_HE is :"<<endl<<endl;
      cout<<estimate.matrixE2H()<<endl;
      if( estimate.reprojectionError( reprojection_error ) )
      {
	cout<<endl<<"Reprojection error: "<<reprojection_error.mean<<" +/- "<<sqrt(reprojection_error.variance);
      }
      if( estimate.transformationError( relative_error ) )
      {
	cout<<endl<<"Relative transformationn error for euler angles: "<<relative_error.euler_angle_error.mean<<" +/- "<<sqrt(relative_error.euler_angle_error.variance);
	cout<<endl<<"Relative transformationn error for translation: "<<relative_error.relative_translation_error.mean<<" +/- "<<sqrt(relative_error.relative_translation_error.variance);
      }
    }
    char egal;
    //std::cin>>egal;
    //rate.sleep();
    //calibrator.printToFile("/home/stewss/Documents/simulation_data.txt");
  }
  
  return 0;
} 
