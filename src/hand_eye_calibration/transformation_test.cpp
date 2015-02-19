 
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
#include <boost/foreach.hpp>

using namespace std;

// code to test hec methods
int main(int argc, char **argv)
{
  using namespace Eigen;
  using namespace st_is;
  using namespace std;
  
  /*
  
  geometry_msgs::Pose eins;
  eins.orientation.x = 0.4303;
  eins.orientation.y = 0.7746;
  eins.orientation.z = -0.1721;
  eins.orientation.w = 0.4303;
  eins.position.x = 3;
  eins.position.y = 1;
  eins.position.z = -5;
  
  Eigen::Matrix<double,3,4> zwei = transformationMatrix(eins);
  */
  /*cout<<endl<<"source:";
  cout<<endl<<eins;
  cout<<endl<<"target:";
  cout<<endl<<zwei;
  */
  /*Eigen::Matrix<double,3,3> rot;
  rot<<0,1,0, -1,0,0, 0,0,1;
  cout<<endl<<"rotation matrix:";
  cout<<endl<<rot;
  Quaterniond q(rot);
  cout<<endl<<"rotation quaternion:";
  cout<<endl<<"x: "<<q.x();
  cout<<endl<<"y: "<<q.y();
  cout<<endl<<"z: "<<q.z();
  cout<<endl<<"w: "<<q.w();
  cout<<endl<<"rotation matrix from quaternion:";
  cout<<endl<<q.toRotationMatrix();
  Quaterniond q_2(0.707106,0,0,0.707106);
  cout<<endl<<"rotation quaternion 2:";
  cout<<endl<<"x: "<<q_2.x();
  cout<<endl<<"y: "<<q_2.y();
  cout<<endl<<"z: "<<q_2.z();
  cout<<endl<<"w: "<<q_2.w();
  cout<<endl<<"rotation matrix from quaternion 2:";
  cout<<endl<<q_2.toRotationMatrix();
  
  Vector3d source(4,2,-3);
  cout<<endl<<"Target of quaternion multiplication:";
  cout<<endl<<source;
  cout<<endl<<"Result of quaternion multiplication:";
  cout<<endl<<q*source;
  cout<<endl<<"Result of untransposed matrix multiplication:";
  cout<<endl<<q.toRotationMatrix()*source;
  cout<<endl<<"vec() output of q:";
  cout<<endl<<q.vec();
  Quaterniond direct_init(0.707106,0,0,0.707106);
  cout<<endl<<"Quaternion from direct initialization:";
  cout<<endl<<"x: "<<direct_init.x();
  cout<<endl<<"y: "<<direct_init.y();
  cout<<endl<<"z: "<<direct_init.z();
  cout<<endl<<"w: "<<direct_init.w();
  cout<<endl<<"Corresponding rotation:";
  cout<<endl<<direct_init.toRotationMatrix();
  
  cout<<endl<<endl;
  
  return 0;*/
  
  ros::init(argc, argv, "autonomous_hand_eye_calibration");
  ros::NodeHandle n("autonomous_hand_eye_calibration");
  
  /*Eigen::Matrix<double,3,3> trans1;
  trans1<<0,1,0, -1,0,0, 0,0,1;
  Eigen::Quaterniond transeye1(trans1);
  cout<<endl<<"trans1 matrix as initialized:";
  cout<<endl<<trans1;
  cout<<endl<<"Quaternion:";
  cout<<endl<<"x: "<<transeye1.x();
  cout<<endl<<"y: "<<transeye1.y();
  cout<<endl<<"z: "<<transeye1.z();
  cout<<endl<<"w: "<<transeye1.w();
  
  double rot_angle = M_PI/2;//2*acos( transeye1.w() );
  Eigen::Vector3d rot_axis( 1, 0, 0 );
  rot_axis.normalize();
  cout<<endl<<"the rotation axis is:";
  cout<<endl<<rot_axis;
  cout<<endl<<"the rotation angle is:";
  cout<<endl<<rot_angle;
  
  Eigen::Matrix<double,3,3> rot_from_q;
  rot_from_q = Eigen::AngleAxisd( rot_angle, rot_axis );
  
  cout<<endl<<"trans1 matrix from quaternion:";
  cout<<endl<<rot_from_q;
  cout<<endl;
  return 0;*/
  //---------------------------------------------------------------
  
  /*Eigen::Matrix<double,3,3> trans;
  trans<<0,1,0, 1,0,0, 0,0,-1;
  Eigen::Quaterniond transeye(trans);
  cout<<endl<<"trans matrix as initialized:";
  cout<<endl<<trans;
  cout<<endl<<"Quaternion:";
  cout<<endl<<"x: "<<transeye.x();
  cout<<endl<<"y: "<<transeye.y();
  cout<<endl<<"z: "<<transeye.z();
  cout<<endl<<"w: "<<transeye.w();
  return 0;*/
  //---------------------------------------------------------------
  
  /*Eigen::Matrix<double,3,4> eye_pose;
  eye_pose<<0,1,0,0, 1,0,0,0, 0,0,-1,8;
  //eye_pose=eye_pose.inverse().eval();
  geometry_msgs::Pose eye;
  eye = st_is::geometryPose(eye_pose);
  cout<<endl<<"eye pose matrix as initialized:";
  cout<<endl<<eye_pose;
  cout<<endl<<"corresponding geometry pose:";
  cout<<endl<<eye<<endl<<endl<<endl<<endl<<endl<<endl;
  return 0;
  */
    
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
  artificial_poses.addNoise( 0.3*M_PI/360, 0.005 );
  //artificial_poses.setSeed( 123 );
  //artificial_poses.calcPosePairs( 1000 );
  //artificial_poses.toFile("/home/stewess/Documents/noise1_pose_set");
  artificial_poses.fromFile("/home/stewess/Documents/gazebo_data_17-2");
  
  TransformationEstimator estimator(&n);
  estimator.setEstimationMethod(daniilidis_1998);
  std::vector<TransformationEstimator::PoseData> poses;
  for( unsigned int i=1;i<4;i++)
  {
    /*hand_eye_calibration::CameraPose cam_pose_request;
    hand_eye_calibration::HandPose hand_pose_request;
    
    ros::Time current_stamp = ros::Time::now();
    
    cam_pose_request.request.request.request_stamp = current_stamp;
    hand_pose_request.request.request.request_stamp = current_stamp;
    
    ros::service::call("/hec/eye_pose",cam_pose_request);
    ros::service::call("/hec/hand_pose",hand_pose_request);
    
    TransformationEstimator::PoseData new_data;
    new_data.eye_pose = cam_pose_request.response.description.pose;
    new_data.hand_pose = hand_pose_request.response.description.pose;
    poses.push_back(new_data);*/
    TransformationEstimator::PoseData new_pose;
    geometry_msgs::Pose eye;
    switch(i)
    {
      case 1:
      {
	Eigen::Matrix<double,3,4> eye_pose;
	eye_pose<<1,0,0,4, 0,1,0,3, 0,0,1,0;
	cout<<endl<<endl<<"eye pose matrix "<<i<<":"<<endl<<eye_pose;
	//eye_pose=eye_pose.inverse().eval();
	eye = st_is::geometryPose(eye_pose);
	break;
      }
      case 2:
      {
	Eigen::Matrix<double,3,4> eye_pose;
	eye_pose<<0,1,0,0, 1,0,0,0, 0,0,-1,8;
	cout<<endl<<endl<<"eye pose matrix "<<i<<":"<<endl<<eye_pose;
	//eye_pose=eye_pose.inverse().eval();
	eye = st_is::geometryPose(eye_pose);
	break;
      }
      case 3:
      {
	Eigen::Matrix<double,3,4> eye_pose;
	eye_pose<<0,0,1,-2, 0,1,0,-3, -1,0,0,5;
	cout<<endl<<endl<<"eye pose matrix "<<i<<":"<<endl<<eye_pose;
	//eye_pose=eye_pose.inverse().eval();
	eye = st_is::geometryPose(eye_pose);
	break;
      }
    };
    cout<<endl<<endl<<"eye pose "<<i<<":"<<endl<<"["<<eye.orientation.x<<" "<<eye.orientation.y<<" "<<eye.orientation.z<<" "<<eye.orientation.w<<"]"<<endl;
    cout<<"["<<eye.position.x<<"; "<<eye.position.y<<"; "<<eye.position.z<<"]"<<endl;
    
    geometry_msgs::Pose hand;
    switch(i)
    {
      case 1:
      {
	Eigen::Matrix<double,3,4> hand_pose;
	hand_pose<<1,0,0,-3, 0,1,0,-2, 0,0,1,0;
	cout<<endl<<endl<<"hand pose matrix "<<i<<":"<<endl<<hand_pose;
	//hand_pose=hand_pose.inverse().eval();
	hand = st_is::geometryPose(hand_pose);
	break;
      }
      case 2:
      {
	Eigen::Matrix<double,3,4> hand_pose;
	hand_pose<<0,1,0,1, 1,0,0,1, 0,0,-1,8;
	cout<<endl<<endl<<"hand pose matrix "<<i<<":"<<endl<<hand_pose;
	//hand_pose=hand_pose.inverse().eval();
	hand = st_is::geometryPose(hand_pose);
	break;
      }
      case 3:
      {
	Eigen::Matrix<double,3,4> hand_pose;
	hand_pose<<0,0,-1,5, 0,1,0,4, 1,0,0,3;
	cout<<endl<<endl<<"hand pose matrix "<<i<<":"<<endl<<hand_pose;
	//hand_pose=hand_pose.inverse().eval();
	hand = st_is::geometryPose(hand_pose);
	break;
      }
    };
    cout<<endl<<endl<<"hand pose "<<i<<":"<<endl<<"["<<hand.orientation.x<<" "<<hand.orientation.y<<" "<<hand.orientation.z<<" "<<hand.orientation.w<<"]"<<endl;
    cout<<"["<<hand.position.x<<"; "<<hand.position.y<<"; "<<hand.position.z<<"]"<<endl;
    
    new_pose.hand_pose = hand;
    new_pose.eye_pose = eye;
    poses.push_back(new_pose);
    //estimator.addNewPosePair();
  }
  
  /*BOOST_FOREACH( TransformationEstimator::PoseData data, poses )
  {
    using namespace std;
    cout<<endl<<"hand:"<<endl<<st_is::transformationMatrix(data.hand_pose);
    cout<<endl<<"eye:"<<endl<<st_is::transformationMatrix(data.eye_pose);
  }*/
  
  DaniilidisDualQuaternionEstimation estimation_method;
  //TransformationEstimator::EstimationData estimation = estimation_method.calculateTransformation( artificial_poses.posePairs() );
  TransformationEstimator::EstimationData estimation = estimation_method.calculateTransformation( poses );
  //TransformationEstimator::EstimationData estimation = estimator.getNewEstimation();
  
  cout<<endl<<"Correct transformation"<<endl<<"-------------------------"<<endl;
  
  cout<<"rotation:"<<endl<<hec_HE.rotation.matrix()<<endl<<endl;
  cout<<"translation:"<<endl<<hec_HE.translation<<endl;
  
  cout<<endl<<"Estimated transformation cam to hand"<<endl<<"-------------------------"<<endl;
  cout<<"rotation:"<<endl<<estimation.rot_HE().matrix()<<endl<<endl;
  cout<<"translation:"<<endl<<estimation.H_trans_HE()<<endl;
  
  /*TransformationEstimator estimator(&n);
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
  cout<<"translation:"<<endl<<estimation.transE2H()<<endl;*/
  cout<<endl<<endl<<"------////////////////////////////"<<endl<<"final test dings und so:"<<endl;
  Quaterniond q(0.1302, 0.007, 0.9915, -0.0013);
  cout<<"quaternion is: [x,y,z,w]: "<<"["<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<"]";
  cout<<"the corresponding matrix according to eigen is:"<<endl;
  cout<<q.toRotationMatrix()<<endl<<endl<<endl;
  
  Quaterniond q2(0.0016,  0.5199,  -0.0020, 0.8542);
  cout<<"q2uaternion is: [x,y,z,w]: "<<"["<<q2.x()<<", "<<q2.y()<<", "<<q2.z()<<", "<<q2.w()<<"]";
  cout<<"the corresponding matrix according to eigen is:"<<endl;
  cout<<q2.toRotationMatrix()<<endl<<endl<<endl;
  
  Quaterniond q3(0.2463, 0, -0.9626, -0.1129);
  cout<<"q3uaternion is: [x,y,z,w]: "<<"["<<q3.x()<<", "<<q3.y()<<", "<<q3.z()<<", "<<q3.w()<<"]";
  cout<<"the corresponding matrix according to eigen is:"<<endl;
  cout<<q3.toRotationMatrix()<<endl<<endl<<endl;
  
  return 0;
} 
