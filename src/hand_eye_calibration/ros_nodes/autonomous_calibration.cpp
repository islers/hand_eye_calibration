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
  while ( calibrator.runSingleIteration() && n.ok() ) // no stop criteria defined yet, it's iterating the whole joint space as defined... gonna take a while :)
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
    //rate.sleep();
    //calibrator.printToFile("/home/stewss/Documents/simulation_data.txt");
  }
  
  program_end: // yes a loop would be prettier but that jump really doesn't hurt anyone here, does it?
  
  cout<<endl<<"The method has finally finished. Would you like to save the result? (y/n)"<<endl;
  char in;
  cin>>in;
  if( in=='n' )
  {
    cout<<endl<<"Are you sure you want to leave without saving? (y/n)";
    char in2;
    cin>>in2;
    if(in2=='y')
      return 0;
  }
  cout<<endl<<"In what file do you want to save the data? (full path)";
  std::string path;
  cin>>path;
  if( calibrator.printToFile(path) )
  {
    cout<<endl<<"Successfully saved data to "<<path<<endl;
    return 0;
  }
  cout<<endl<<"Really sorry but something went wrong."<<endl;
  
  goto program_end;
  
  
  return 0;
} 
