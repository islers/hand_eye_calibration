/* Copyright (c) 2014, 2015, Stefan Isler, islerstefan@bluewin.ch
 * Gajamohan Mohanarajah and Ferrara Francesco
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

#include "hand_eye_calibration/park_1994_estimation.h" 
#include "hand_eye_calibration/estimation_data.h" 
#include <Eigen/StdVector>

#include "utils/eigen_utils.h"
#include "utils/math.h"
#include "utils/ros_eigen.h"

using namespace std;
using namespace Eigen;


ParkHECEstimation::ParkHECNewPtrConstructor park_1994;


std::string ParkHECEstimation::estimationMethod()
{
  return g_method_name_;
}

std::string ParkHECEstimation::g_method_name_ = "park_1994";

#include <boost/foreach.hpp>
TransformationEstimator::EstimationData ParkHECEstimation::calculateTransformation( std::vector<TransformationEstimator::PoseData>& _pose_data, bool _suppressWarnings )
{
  ROS_INFO("Calculating hand-eye transformation...");
    
  if( _pose_data.size()<=2 ) /// POSSIBLE FIVE?! ->CHECK OUT //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
  {
    std::stringstream msg;
    msg<<"ParkHECEstimation::calculateTransformation:: At least three hand-eye pose pairs are necessary for computation."<<"Number of pose pairs currently available for computation: "<<(int)_pose_data.size()<<" Aborting transformation computation.";
    ROS_ERROR_STREAM(msg.str());
    throw std::runtime_error(msg.str());
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
  /// FIT DATA TYPES... //////////////////////////////////////////////////////////////////////
  //least squares estimation
  /*Matrix3d M;
  Matrix4d rbi, rbj, cbi, cbj;
  Matrix4d A, B;

  Matrix3d I;
  I.setIdentity();
  MatrixXf C(0,3), bA(0,1), bB(0,1);

  Vector3d ai, bi;

  VectorXd V_tmp;
  MatrixXf C_tmp;

  M.setZero();

  for(int i=0; i < (int)rotationRB_vec.size(); i++){
    for(int j=0; j < (int)rotationRB_vec.size(); j++){
    if(i!=j)
    {
      rbi << rotationRB_vec[i] , translationRB_vec[i] ,  0, 0, 0, 1;
      rbj << rotationRB_vec[j] , translationRB_vec[j] ,  0, 0, 0, 1;
      A = rbj.inverse()*rbi;

      cbi << rotationCB_vec[i] , translationCB_vec[i] ,  0, 0, 0, 1;
      cbj << rotationCB_vec[j] , translationCB_vec[j] ,  0, 0, 0, 1;
      B = cbj*cbi.inverse();

      ai = getLogTheta(A.block(0,0,3,3));
      bi = getLogTheta(B.block(0,0,3,3));

      M += bi*ai.transpose();

      MatrixXf C_tmp = C;
      C.resize(C.rows()+3, NoChange);
      C << C_tmp,  Matrix3d::Identity() - A.block(0,0,3,3);

      V_tmp = bA;
      bA.resize(bA.rows()+3, NoChange);
      bA << V_tmp, A.block(0,3,3,1);

      V_tmp = bB;
      bB.resize(bB.rows()+3, NoChange);
      bB << V_tmp, B.block(0,3,3,1);

      }//end of if i!=j
    }
    
  }//end of for(.. i < rotationRB_vec.size(); ..)

  EigenSolver<Matrix3d> es(M.transpose()*M);
  Matrix3cd D = es.eigenvalues().asDiagonal();
  Matrix3cd V = es.eigenvectors();

  Matrix3cd Lambda = D.inverse().array().sqrt();
  Matrix3cd Theta_X = V * Lambda * V.inverse() * M.transpose();
  std::cout << "Orientation of Camera Frame with respect to Robot tool-tip frame." << std::endl;
  std::cout << "Theta_X = [ " << Theta_X.real()  << " ]; " << endl;

  //Estimating translational offset
  for(int i=0; i < bB.rows()/3; i++)
  {
    bB.block(i*3,0,3,1) = Theta_X.real()*bB.block(i*3,0,3,1);
  }
  bA = bA - bB; // this is d. saving memory

  std::cout << "Translation of Camera Frame with respect to Robot tool-tip frame." << std::endl;
  cout << "bX = [ " << (C.transpose()*C).inverse() * C.transpose() * bA << " ]; " << endl;
  
    
  TransformationEstimator::EstimationData new_estimate( estimationMethod(), rot_EH, E_trans_EH );
  
  return new_estimate;*/
}

TransformationEstimator::EstimationData ParkHECEstimation::operator()( std::vector<TransformationEstimator::PoseData>& _pose_data, bool _suppressWarnings )
{
  return calculateTransformation( _pose_data, _suppressWarnings );
}