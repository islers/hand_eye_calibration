 
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

// code to test hec methods
int main(int argc, char **argv)
{
  using namespace Eigen;
  using namespace st_is;
  
  ros::init(argc, argv, "autonomous_hand_eye_calibration");
  ros::NodeHandle n("autonomous_hand_eye_calibration");
  /*  
  // define relative pose of world to robot base
  Eigen::Matrix3d a1;
  a1 = AngleAxisd(1.23*M_PI, Vector3d::UnitY() )
  * Eigen::AngleAxisd( 5.12*M_PI, Eigen::Vector3d::UnitY() )
  * Eigen::AngleAxisd( -0.2*M_PI, Eigen::Vector3d::UnitZ() );
  Eigen::Quaterniond r1(a1);
  Eigen::Vector3d t1(-2.0006031,1.6234,1.23);
  CoordinateTransformation t_WB(r1,t1);
    
  // define hand-eye transformation
  AngleAxisd hec_rot_aa(5, Vector3d::UnitX() );
  Eigen::Quaterniond hec_rot(hec_rot_aa);
  Eigen::Vector3d hec_trans(1,-3,10);
  
  CoordinateTransformation hec_HE(hec_rot,hec_trans);
  
  PoseCreator artificial_poses( t_WB, hec_HE );
  artificial_poses.addNoise( M_PI/360, 0.02 );
  //artificial_poses.setSeed( 123 );
  artificial_poses.calcPosePairs( 5000 );
  artificial_poses.toFile("/home/stewess/Documents/noise1_pose_set");
  //artificial_poses.fromFile("/home/stewess/Documents/pose_set");
  
  DaniilidisDualQuaternionEstimation estimation_method;
  TransformationEstimator::EstimationData estimation = estimation_method.calculateTransformation( artificial_poses.posePairs() );
  
  cout<<endl<<"Correct transformation"<<endl<<"-------------------------"<<endl;
  
  cout<<"rotation:"<<endl<<hec_HE.rotation.matrix()<<endl<<endl;
  cout<<"translation:"<<endl<<hec_HE.translation<<endl;
  
  cout<<endl<<"Estimated transformation"<<endl<<"-------------------------"<<endl;
  cout<<"rotation:"<<endl<<estimation.rot_HE().matrix()<<endl<<endl;
  cout<<"translation:"<<endl<<estimation.transH2E()<<endl;
  */
  TransformationEstimator estimator(&n);
  estimator.loadFromFile( "/home/stewess/Documents/gazebo-hec-07-42.txt" );
  estimator.setEstimationMethod( daniilidis_1998 );
  
  std::vector<TransformationEstimator::PoseData> pose_data;
  pose_data = estimator.poseData();
  
  PoseCreator interface;
  interface.setPosePairs(pose_data);
  interface.toFile("/home/stewess/Documents/gazebo-hec-07-42");
  
  TransformationEstimator::EstimationData estimation = estimator.estimate();
  
  cout<<endl<<"Estimated transformation"<<endl<<"-------------------------"<<endl;
  cout<<"rotation:"<<endl<<estimation.rot_HE().matrix()<<endl<<endl;
  cout<<"translation:"<<endl<<estimation.transE2H()<<endl;
  
  
  return 0;
} 
