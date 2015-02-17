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

#include "hand_eye_calibration/dual_quaternion_transformation_estimator.h" 
#include "hand_eye_calibration/estimation_data.h" 
#include <Eigen/StdVector>

#include "utils/eigen_utils.h"
#include "utils/math.h"
#include "utils/ros_eigen.h"

using namespace std;
using namespace Eigen;


DaniilidisDualQuaternionEstimation::DaniilidisNewPtrConstructor daniilidis_1998;


std::string DaniilidisDualQuaternionEstimation::estimationMethod()
{
  return g_method_name_;
}

std::string DaniilidisDualQuaternionEstimation::g_method_name_ = "dual_quaternion_estimation_daniilidis_1998";

#include <boost/foreach.hpp>
TransformationEstimator::EstimationData DaniilidisDualQuaternionEstimation::calculateTransformation( std::vector<TransformationEstimator::PoseData>& _pose_data, bool _suppressWarnings )
{
  ROS_INFO("Calculating hand-eye transformation...");
    
  if( _pose_data.size()<=1 )
  {
    std::stringstream msg;
    msg<<"DaniilidisDualQuaternionEstimation::calculateTransformation:: At least two hand-eye pose pairs are necessary for computation."<<"Number of pose pairs currently available for computation: "<<(int)_pose_data.size()<<" Aborting transformation computation.";
    ROS_ERROR_STREAM(msg.str());
    throw std::runtime_error(msg.str());
  }
  else if( _pose_data.size()<20 && !_suppressWarnings )
  {
    ROS_WARN("For good results more than 20 hand-eye pose pairs are recommended. The computed transformation might be inaccurate.");
    ROS_INFO("Number of pose pairs currently available for computation: %i", (int)_pose_data.size() );
    ROS_INFO("Going on with calculation...");
  }
  
  /*for( auto i=0;i<_pose_data.size();i++ )
  {
    if(i==1)
    {
      _pose_data[i].hand_pose.orientation.x = -_pose_data[i].hand_pose.orientation.x;
      _pose_data[i].hand_pose.orientation.y = -_pose_data[i].hand_pose.orientation.y;
      _pose_data[i].hand_pose.orientation.z = -_pose_data[i].hand_pose.orientation.z;
      _pose_data[i].hand_pose.orientation.w = -_pose_data[i].hand_pose.orientation.w;
    }
  }*/
  
  /*BOOST_FOREACH( auto pose, _pose_data )
  {
    cout<<endl<<"hand pose";
    cout<<endl<<pose.hand_pose;
    cout<<endl<<endl<<"eye pose";
    cout<<endl<<pose.eye_pose;
  }*/
  
  /* calculate movement transformations from pose to pose from base<->pose transformations and calculate
   * dual quaternions
   * input:	- hand: transformation base->hand
   * 		- eye:	transformation grid->camera
   * 
   * 
   *  dual quaternions are then hQ[i].first + epsilon*hQ[i].second
  */
  vector<st_is::DualQuaternion, Eigen::aligned_allocator<st_is::DualQuaternion> > hQ, cQ;
  for( int i=0; i<_pose_data.size()-1; i++ )
  {
    // transformation hand pose 1 (H1) -_> hand pose 2 (H2)
    
    Quaterniond rot_BH1 = st_is::geometryToEigen( _pose_data[i].hand_pose.orientation );
    Vector3d B_trans_BH1 = st_is::geometryToEigen( _pose_data[i].hand_pose.position );
    Quaterniond rot_H2B = st_is::geometryToEigen( _pose_data[i+1].hand_pose.orientation ).inverse();
    Vector3d B_trans_BH2 = st_is::geometryToEigen( _pose_data[i+1].hand_pose.position );
    
    Quaterniond rot_H2H1 = rot_H2B * rot_BH1;
    Vector3d H2_trans_H2H1 = rot_H2B * ( B_trans_BH1-B_trans_BH2 );
    st_is::DualQuaternion dualQuat_H2H1( rot_H2H1, H2_trans_H2H1 );
    
    geometry_msgs::Pose pose;
    pose.orientation = st_is::eigenToGeometry(rot_H2H1);
    pose.position = st_is::eigenToGeometry(H2_trans_H2H1);
    //cout<<endl<<"transformation matrix:";
    //cout<<endl<<st_is::transformationMatrix(pose);
    //cout<<endl<<"q:["<<dualQuat_H2H1.q.x()<<", "<<dualQuat_H2H1.q.y()<<", "<<dualQuat_H2H1.q.z()<<", "<<dualQuat_H2H1.q.w()<<"]";
    //cout<<endl<<"q':["<<dualQuat_H2H1.q_prime.x()<<", "<<dualQuat_H2H1.q_prime.y()<<", "<<dualQuat_H2H1.q_prime.z()<<", "<<dualQuat_H2H1.q_prime.w()<<"]";
    
    hQ.push_back( dualQuat_H2H1 );
    
    //transformation cam pose 1 (E1) -> cam pose 2 (E2)
    
    Quaterniond rot_GC1 = st_is::geometryToEigen( _pose_data[i].eye_pose.orientation ).inverse();
    Vector3d C1_trans_C1G = st_is::geometryToEigen( _pose_data[i].eye_pose.position );
    Quaterniond rot_C2G = st_is::geometryToEigen( _pose_data[i+1].eye_pose.orientation );
    Vector3d C2_trans_C2G = st_is::geometryToEigen( _pose_data[i+1].eye_pose.position );
    
    Quaterniond rot_C2C1 = rot_C2G * rot_GC1;
    Vector3d C2_trans_C2C1 = C2_trans_C2G - rot_C2C1*C1_trans_C1G;
    st_is::DualQuaternion dualQuat_C2C1( rot_C2C1, C2_trans_C2C1 );
    
    //cout<<endl<<"q:["<<dualQuat_C2C1.q.x()<<", "<<dualQuat_C2C1.q.y()<<", "<<dualQuat_C2C1.q.z()<<", "<<dualQuat_C2C1.q.w()<<"]";
    //cout<<endl<<"q':["<<dualQuat_C2C1.q_prime.x()<<", "<<dualQuat_C2C1.q_prime.y()<<", "<<dualQuat_C2C1.q_prime.z()<<", "<<dualQuat_C2C1.q_prime.w()<<"]";
        
    cQ.push_back( dualQuat_C2C1 );
    
    if( dualQuat_H2H1.q.w()<0 || dualQuat_C2C1.q.w()<0 )
    {
      cout<<endl<<"H2H1.q: "<<dualQuat_H2H1.q.w()<<" , C2C1.q: "<<dualQuat_C2C1.q.w();
      cout<<endl<<"H2H1.q_prime: "<<dualQuat_H2H1.q_prime.w()<<" , C2C1.q_prime: "<<dualQuat_C2C1.q_prime.w();
      cout<<endl<<"------------------------------"<<endl;
    }
  }
  
  // build  S(i)...
  vector< Matrix<double,6,8>, Eigen::aligned_allocator<Eigen::Matrix<double,6,8> > > S;
  S.resize( hQ.size() );
  
  
  for( unsigned int i=0; i<hQ.size(); i++ )
  {
    Matrix<double,6,8> S_i;
    S[i] <<	hQ[i].q.vec()-cQ[i].q.vec()		,st_is::crossProdMatrix( hQ[i].q.vec()+cQ[i].q.vec() )		,Matrix<double,3,1>::Zero()			,Matrix<double,3,3>::Zero()
		,hQ[i].q_prime.vec()-cQ[i].q_prime.vec()		,st_is::crossProdMatrix( hQ[i].q_prime.vec()+cQ[i].q_prime.vec() )	,hQ[i].q.vec()-cQ[i].q.vec()		,st_is::crossProdMatrix( hQ[i].q.vec()+cQ[i].q.vec());
  }
  /*cout<<endl<<endl;
  cout<<endl<<S[0];
  cout<<endl<<S[1];
  cout<<endl<<endl;*/
  // build T
  Matrix<double,Dynamic,Dynamic> T; // 8*x-matrix
  for( unsigned int i=0; i<hQ.size(); i++ )
  {
    T.conservativeResize( 8, T.cols()+6 );
    T.block<8,6>(0,T.cols()-6) << S[i].transpose();
  }
  
  T.transposeInPlace();
  
  // null space of T: right null vectors
  JacobiSVD<MatrixXd,FullPivHouseholderQRPreconditioner> svd( T, ComputeFullV );
  MatrixXd V = svd.matrixV();
  
  
  Matrix<double,8,1> v_7 = V.col(6);
  Matrix<double,8,1> v_8 = V.col(7);
  
  
  /* decompose 8x1 v-vectors into two 4x1 vectors for
  /* 
  /* lambda_1*v_7 + lambda_2*v_8 = q; q: searched dual quaternion describing the hand-eye transformation
   * 
   * then
   * 
   * it follows from the constrains q^T*q = 1 and q^T*q' = 0:
   * 
   * lambda_1² * u_1^T * u_1 + 2*lambda_1*lambda_2*u_1^T*u_2 + lambda_2² * u_2^T * u_2 = 1
   * and
   * lambda² * u_1^T * v_1 + lambda_1*lambda_2*(u_1^T*v_2+u_2^T*v_1) + lambda_2² * u_2^T * v_2 = 0
   * 
   * with s = lambda_1 / lambda_2
   * 
   * lambda_1²/lambda_2² * u_1^T * v_1 + lambda_1*lambda_2/lambda_2² * (u_1^T * v_2 + u_2^T * v_1 ) + lambda_2² / lambda_2² * u_2^T * v_1 = 0
   * s² * u_1^T * v_1 + s * (u_1^T * v_2 + u_2^T * v_1 ) + u_2^T*v_2 = 0
   */
  
  Matrix<double,4,1> u_1 = v_7.topRows(4);
  Matrix<double,4,1> v_1 = v_7.bottomRows(4);
  
  Matrix<double,4,1> u_2 = v_8.topRows(4);
  Matrix<double,4,1> v_2 = v_8.bottomRows(4);
  
  // solve the quadratic equation for s
  pair<double,double> s;
  double a = (u_1.transpose() * v_1)[0];
  double b = (u_1.transpose() * v_2 + u_2.transpose() * v_1)[0];
  double c = (u_2.transpose() * v_2)[0];
  
  // equation has two real solutions of opposite sign (see paper)
  st_is::roots(a,b,c,s);
  //cout<<endl<<"s.first="<<s.first;
  //cout<<endl<<"s.second="<<s.second;
  
  // using 2nd equation:
  double leftSideCandidate_1 = (s.first*s.first*u_1.transpose()*u_1 + 2*s.first*u_1.transpose()*u_2 + u_2.transpose()*u_2)[0];
  double leftSideCandidate_2 = (s.second*s.second*u_1.transpose()*u_1 + 2*s.second*u_1.transpose()*u_2 + u_2.transpose()*u_2)[0];
  
  // use larger value
  double sSol, leftSide;
  if( leftSideCandidate_1>leftSideCandidate_2 )
  {
    sSol = s.first;
    leftSide = leftSideCandidate_1;
  }
  else
  {
    sSol = s.second;
    leftSide = leftSideCandidate_2;
  }
  
  // calculate lambdas
  double lambda_2 = sqrt(1/leftSide);
  double lambda_1 = sSol * lambda_2;
  
  //cout<<endl<<"leftSide="<<leftSide;
  //cout<<endl<<"lambda_2="<<lambda_2;
  //cout<<endl<<"lambda_1="<<lambda_1;
  
  // finally calculate resulting dual quaternion (w, x, y, z)^T
  Matrix<double,8,1> qVec = lambda_1 * v_7 + lambda_2 * v_8;
  
  pair<Quaterniond,Quaterniond> q;
  q.first = Quaterniond( qVec(0),-qVec(1),-qVec(2),-qVec(3) ); // quaternion representing the rotation R_CH (hand to eye) : negatives because Eigen defines Quaternions differently
  q.second = Quaterniond( qVec(4), -qVec(5), -qVec(6), -qVec(7) );
  
  //using namespace std;
  //cout<<endl<<v_7;
  //cout<<endl<<v_8;
  //cout<<endl<<"q:["<<qVec(0)<<", "<<qVec(1)<<", "<<qVec(2)<<", "<<qVec(3)<<"]";
  //cout<<endl<<"q':["<<qVec(4)<<", "<<qVec(5)<<", "<<qVec(6)<<", "<<qVec(7)<<"]";
  
  Quaterniond rot_EH = q.first;
  // calculate the translation
  Quaterniond t_p = q.second*q.first.conjugate();
  Vector3d t = 2*t_p.vec(); // translation vector representing the translation from hand- to eye-frame in hand coordinates
  
  // build output transformation
  Eigen::Vector3d E_trans_EH = t;
  
    
  TransformationEstimator::EstimationData new_estimate( estimationMethod(), rot_EH, E_trans_EH );
  
  return new_estimate;
}

TransformationEstimator::EstimationData DaniilidisDualQuaternionEstimation::operator()( std::vector<TransformationEstimator::PoseData>& _pose_data, bool _suppressWarnings )
{
  return calculateTransformation( _pose_data, _suppressWarnings );
}