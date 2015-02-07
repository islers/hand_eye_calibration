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
#include <random>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomous_hand_eye_calibration");
  ros::NodeHandle n("autonomous_hand_eye_calibration");
  
  using namespace Eigen;
  using namespace st_is;
  
  
  // define relative pose of world to robot base
  AngleAxisd a1(M_PI, Vector3d::UnitY() );
  Eigen::Quaterniond r1(a1);
  Eigen::Vector3d t1(-2.0006031,1.6234,1.23);
  CoordinateTransformation t_WB(r1,t1);
    
  // define hand-eye transformation
  AngleAxisd hec_rot_aa(0, Vector3d::UnitX() );
  Eigen::Quaterniond hec_rot(hec_rot_aa);
  Eigen::Vector3d hec_trans(1,0,0);
  
  CoordinateTransformation hec_HE(hec_rot,hec_trans);
  
  
  std::mt19937 random_number_generator; // mersenne twister random number generator
  std::uniform_real_distribution<double> rotations(-2,2);
  std::uniform_real_distribution<double> translation(0.001,2.000);
  
  // create random hand-eye pose set
  std::vector<TransformationEstimator::PoseData> pose_set;
  for( int i = 0; i<10; i++ )
  {
    double r_angle_1 = rotations(random_number_generator);
    double r_angle_2 = rotations(random_number_generator);
    double r_angle_3 = rotations(random_number_generator);
    double r_transl_1 = translation(random_number_generator);
    double r_transl_2 = translation(random_number_generator);
    double r_transl_3 = translation(random_number_generator);
    // define eye pose
    Matrix3d m;
    m = AngleAxisd(r_angle_1*M_PI, Vector3d::UnitX())
    * AngleAxisd(r_angle_2*M_PI, Vector3d::UnitY())
    * AngleAxisd(r_angle_3*M_PI, Vector3d::UnitZ());
    
    Eigen::Quaterniond eye_rot(m);
    Eigen::Vector3d eye_trans(r_transl_1,r_transl_2,r_transl_3);
    CoordinateTransformation t_EW(eye_rot,eye_trans);
    
    
    // calculate corresponding hand pose relative to base
    CoordinateTransformation t_BH = t_WB.inv()*t_EW.inv()*hec_HE.inv();
    
    TransformationEstimator::PoseData pose;
    pose.hand_pose.orientation = eigenToGeometry(t_BH.rotation);
    pose.hand_pose.position = eigenToGeometry(t_BH.translation);
    pose.eye_pose.orientation = eigenToGeometry(t_EW.rotation);
    pose.eye_pose.position = eigenToGeometry(t_EW.translation);
    
    cout<<endl<<"hand_pose"<<endl<<"-------------------------"<<endl;
    cout<<"rotation:"<<endl<<t_BH.rotation.matrix()<<endl;
    cout<<"translation:"<<endl<<t_BH.translation<<endl;
    
    cout<<endl<<"eye_pose"<<endl<<"-------------------------"<<endl;
    cout<<"rotation:"<<endl<<t_EW.rotation.matrix()<<endl;
    cout<<"translation:"<<endl<<t_EW.translation<<endl;
    
  
    pose_set.push_back(pose);
  }
  
  DaniilidisDualQuaternionEstimation estimation_method;
  TransformationEstimator::EstimationData estimation = estimation_method.calculateTransformation(pose_set);
  
  cout<<endl<<"Correct transformation"<<endl<<"-------------------------"<<endl;
  cout<<"rotation:"<<endl<<hec_HE.rotation.matrix()<<endl;
  cout<<"translation:"<<endl<<hec_HE.translation<<endl;
  
  /*cout<<endl<<"Correct transformation calculated backwards"<<endl<<"-------------------------"<<endl;
  CoordinateTransformation t_HE_check = (t_EW*t_WB*t_BH).inv(); //t_BH.inv()*t_WB.inv()*t_EW.inv(); 
  cout<<"rotation:"<<endl<<t_HE_check.rotation.matrix()<<endl;
  cout<<"translation:"<<endl<<t_HE_check.translation<<endl;*/
  
  cout<<endl<<"Estimated transformation"<<endl<<"-------------------------"<<endl;
  cout<<"rotation:"<<endl<<estimation.rot_HE().matrix()<<endl;
  cout<<"translation:"<<endl<<estimation.transH2E()<<endl;
  //AutonomousHandEyeCalibrator calibrator(&n);
  
  
  /*
  ros::Rate rate(0.2);
  while ( calibrator.runSingleIteration() && n.ok() )
  {
    rate.sleep();
  }*/
  
  return 0;
} 
