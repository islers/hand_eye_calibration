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

#include "hand_eye_calibration/pose_creator.h"
#include "utils/ros_eigen.h"
#include <random>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

PoseCreator::PoseCreator():
  random_seed_(true),
  hand_rot_stddev_(0),
  hand_trans_stddev_(0),
  eye_rot_stddev_(0),
  eye_trans_stddev_(0)
{
  std::random_device rd;
  std::mt19937 random_number_generator(rd()); // mersenne twister random number generator with random seed
  std::uniform_real_distribution<double> rotations(-2,2);
  std::uniform_real_distribution<double> translation(0.0,4.000);
  
  // world-base-transform creation
  Eigen::Matrix3d m_WB;
  m_WB = Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitX())
  * Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitZ());
  
  Eigen::Quaterniond rot_WB(m_WB);
  Eigen::Vector3d trans_WB(translation(random_number_generator),translation(random_number_generator),translation(random_number_generator));
  t_WB_ = st_is::CoordinateTransformation(rot_WB,trans_WB);
  
  // hand-eye-transform creation
  Eigen::Matrix3d m_HE;
  m_HE = Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitX())
  * Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(rotations(random_number_generator)*M_PI, Eigen::Vector3d::UnitZ());
  
  Eigen::Quaterniond rot_HE(m_HE);
  Eigen::Vector3d trans_HE(translation(random_number_generator),translation(random_number_generator),translation(random_number_generator));
  t_HE_ = st_is::CoordinateTransformation(rot_HE,trans_HE);
}

PoseCreator::PoseCreator( st_is::CoordinateTransformation _t_WB, st_is::CoordinateTransformation _t_HE ):
  random_seed_(true),
  hand_rot_stddev_(0),
  hand_trans_stddev_(0),
  eye_rot_stddev_(0),
  eye_trans_stddev_(0)
{
  t_WB_ = _t_WB;
  t_HE_ = _t_HE;
}

PoseCreator::PoseCreator( geometry_msgs::Pose _t_WB, geometry_msgs::Pose _t_HE ):
  random_seed_(true),
  hand_rot_stddev_(0),
  hand_trans_stddev_(0),
  eye_rot_stddev_(0),
  eye_trans_stddev_(0)
{
  t_WB_ = st_is::CoordinateTransformation();
  t_HE_ = st_is::CoordinateTransformation();
  
  t_WB_.rotation = st_is::geometryToEigen( _t_WB.orientation );
  t_WB_.translation = st_is::geometryToEigen( _t_WB.position );
  t_HE_.rotation = st_is::geometryToEigen( _t_HE.orientation );
  t_HE_.translation = st_is::geometryToEigen( _t_HE.position );
}

PoseCreator::PoseCreator( Eigen::Matrix<double,3,4> _t_WB, Eigen::Matrix<double,3,4> _t_HE ):
  random_seed_(true),
  hand_rot_stddev_(0),
  hand_trans_stddev_(0),
  eye_rot_stddev_(0),
  eye_trans_stddev_(0)
{
  t_WB_.rotation = Eigen::Quaterniond( _t_WB.topRightCorner<3,3>() );
  t_WB_.translation = _t_WB.rightCols(1);
  t_HE_.rotation = Eigen::Quaterniond( _t_HE.topRightCorner<3,3>() );
  t_HE_.translation = _t_HE.rightCols(1);
}

void PoseCreator::setWorldBaseTransform( geometry_msgs::Pose _t_WB )
{
  t_WB_ = st_is::CoordinateTransformation();
  t_WB_.rotation = st_is::geometryToEigen( _t_WB.orientation );
  t_WB_.translation = st_is::geometryToEigen( _t_WB.position );
}

void PoseCreator::setWorldBaseTransform( Eigen::Matrix<double,3,4> _t_WB )
{
  t_WB_.rotation = Eigen::Quaterniond( _t_WB.topRightCorner<3,3>() );
  t_WB_.translation = _t_WB.rightCols(1);
}

void PoseCreator::setHandEyeTransform( geometry_msgs::Pose _t_HE )
{
  t_HE_ = st_is::CoordinateTransformation();
  t_HE_.rotation = st_is::geometryToEigen( _t_HE.orientation );
  t_HE_.translation = st_is::geometryToEigen( _t_HE.position );
}

void PoseCreator::setHandEyeTransform( Eigen::Matrix<double,3,4> _t_HE )
{
  t_HE_.rotation = Eigen::Quaterniond( _t_HE.topRightCorner<3,3>() );
  t_HE_.translation = _t_HE.rightCols(1);
}

void PoseCreator::getWorldBaseTransform( geometry_msgs::Pose& _t_WB )
{
  _t_WB.orientation = st_is::eigenToGeometry( t_WB_.rotation );
  _t_WB.position = st_is::eigenToGeometry( t_WB_.translation );
}

void PoseCreator::getWorldBaseTransform( Eigen::Matrix<double,3,4>& _t_WB )
{
  _t_WB.topRightCorner<3,3>() = t_WB_.rotation.matrix();
  _t_WB.rightCols(1) = t_WB_.translation;
}

void PoseCreator::getWorldBaseTransform( st_is::CoordinateTransformation& _t_WB )
{
  _t_WB = t_WB_;
}

void PoseCreator::getHandEyeTransform( geometry_msgs::Pose& _t_HE )
{
  _t_HE.orientation = st_is::eigenToGeometry( t_HE_.rotation );
  _t_HE.position = st_is::eigenToGeometry( t_HE_.translation );
}

void PoseCreator::getHandEyeTransform( Eigen::Matrix<double,3,4>& _t_HE )
{
  _t_HE.topRightCorner<3,3>() = t_HE_.rotation.matrix();
  _t_HE.rightCols(1) = t_HE_.translation;
}

void PoseCreator::getHandEyeTransform( st_is::CoordinateTransformation& _t_HE )
{
  _t_HE = t_HE_;
}

std::vector<TransformationEstimator::PoseData>& PoseCreator::posePairs()
{
  return pose_pairs_;
}

std::vector<TransformationEstimator::PoseData>& PoseCreator::calcPosePairs( int _number_of_pairs, std::pair<double,double> _rotation_bounds, std::pair<double,double> _translation_bounds )
{
  std::mt19937 random_number_generator; // mersenne twister random number generator
  std::uniform_real_distribution<double> rotations(-2,2);
  std::uniform_real_distribution<double> translation(0.001,2.000);
  
  if( random_seed_ )
  {
    std::random_device rd;
    random_number_generator.seed( rd() );
  }
  else
    random_number_generator.seed( seed_value_ );
  
  std::uniform_real_distribution<double> rot( _rotation_bounds.first, _rotation_bounds.second );
  std::uniform_real_distribution<double> trans( _translation_bounds.first, _translation_bounds.second );
    
  // create random hand-eye pose set
  for( int i = 0; i < _number_of_pairs; i++ )
  {
    // create random eye pose through euler angles
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd( rot(random_number_generator)*M_PI, Eigen::Vector3d::UnitX() )
    * Eigen::AngleAxisd( rot(random_number_generator)*M_PI, Eigen::Vector3d::UnitY() )
    * Eigen::AngleAxisd( rot(random_number_generator)*M_PI, Eigen::Vector3d::UnitZ() );
    
    Eigen::Quaterniond eye_rot(m);
    Eigen::Vector3d eye_trans( trans(random_number_generator), trans(random_number_generator), trans(random_number_generator) );
    st_is::CoordinateTransformation t_EW( eye_rot,eye_trans );
    
    // calculate corresponding hand pose relative to base
    st_is::CoordinateTransformation t_BH = t_WB_.inv()*t_EW.inv()*t_HE_.inv();
    
    TransformationEstimator::PoseData pose;
    pose.hand_pose.orientation = st_is::eigenToGeometry( Eigen::Quaterniond(rotationNoise(hand_rot_stddev_)*t_BH.rotation) );
    pose.hand_pose.position = st_is::eigenToGeometry(t_BH.translation + translationNoise(hand_trans_stddev_));
    pose.eye_pose.orientation = st_is::eigenToGeometry( Eigen::Quaterniond(rotationNoise(eye_rot_stddev_)*t_EW.rotation) );
    pose.eye_pose.position = st_is::eigenToGeometry(t_EW.translation + translationNoise(eye_trans_stddev_));
    
    pose_pairs_.push_back(pose);
  }
  
}

void PoseCreator::addNoise( double _rotation_noise, double _translation_noise )
{
  hand_rot_stddev_ = _rotation_noise;
  eye_rot_stddev_ = _rotation_noise;
  hand_trans_stddev_ = _translation_noise;
  eye_trans_stddev_ = _translation_noise;
}

void PoseCreator::addHandNoise( double _rotation_noise, double _translation_noise )
{
  hand_rot_stddev_ = _rotation_noise;
  hand_trans_stddev_ = _translation_noise;
}

void PoseCreator::addEyeNoise( double _rotation_noise, double _translation_noise )
{
  eye_rot_stddev_ = _rotation_noise;
  eye_trans_stddev_ = _translation_noise;
}

void PoseCreator::setSeed( int _seed_value )
{
  random_seed_ = false;
  seed_value_ = _seed_value;
}

void PoseCreator::setRandomSeed()
{
  random_seed_ = true;
}

void PoseCreator::toFile( std::string _filename )
{  
  std::ofstream hand_poses( _filename+"_hand.txt" );
  std::ofstream eye_poses( _filename+"_eye.txt" );
  
  for( unsigned int i=0; i<pose_pairs_.size(); i++ )
  {
    Eigen::Matrix<double,3,4> hand_pose_matrix;
    hand_pose_matrix.topLeftCorner<3,3>() = st_is::geometryToEigen( pose_pairs_[i].hand_pose.orientation ).matrix();
    hand_pose_matrix.rightCols(1) = st_is::geometryToEigen( pose_pairs_[i].hand_pose.position );
  
    Eigen::Matrix<double,3,4> eye_pose_matrix;
    eye_pose_matrix.topLeftCorner<3,3>() = st_is::geometryToEigen( pose_pairs_[i].eye_pose.orientation ).matrix();
    eye_pose_matrix.rightCols(1) = st_is::geometryToEigen( pose_pairs_[i].eye_pose.position );
    
    hand_poses << hand_pose_matrix;
    if( i!=pose_pairs_.size()-1 ) hand_poses << std::endl;
    eye_poses << eye_pose_matrix;
    if( i!=pose_pairs_.size()-1 ) eye_poses << std::endl;
  }
}

void PoseCreator::fromFile( std::string _filename )
{
  pose_pairs_.clear(); // delete old data
  
  std::ifstream hand_pose_file( _filename+"_hand.txt" );
  std::ifstream eye_pose_file( _filename+"_eye.txt" );
  
  std::vector<geometry_msgs::Pose> hand_poses;
  std::vector<geometry_msgs::Pose> eye_poses;
  
  double data_read;
  while( true )
  {
    Eigen::Matrix<double,3,4> hand_pose_matrix;
    bool calculation_finished = false;
    
    for( int i = 0; i<12; i++ )
    {
      if( hand_pose_file >> data_read )
      {
	int col = i%4;
	int row = (i-col)/4;
	hand_pose_matrix(row,col) = data_read;
      }
      else
      {
	if( i!=0 ) // attempt to read new matrix that doesn't exist
	{
	  std::cout<<std::endl<<"i is "<<i<<std::endl;
	  std::runtime_error e("PoseCreator::fromFile::The number of elements in the hand pose file isn't valid. Nothing is read.");
	  throw e;
	}
	else
	{
	  calculation_finished = true;
	  break;
	}
      }
    }
    if( calculation_finished )
      break;
    
    geometry_msgs::Pose hand_pose;
    hand_pose.orientation = st_is::eigenToGeometry( Eigen::Quaterniond( hand_pose_matrix.topLeftCorner<3,3>() ) );
    hand_pose.position = st_is::eigenToGeometry( Eigen::Vector3d(hand_pose_matrix.rightCols(1)) );;
    hand_poses.push_back( hand_pose );
  }
  while( true )
  {
    Eigen::Matrix<double,3,4> eye_pose_matrix;
    bool calculation_finished = false;
    
    for( int i = 0; i<12; i++ )
    {
      if( eye_pose_file >> data_read )
      {
	int col = i%4;
	int row = (i-col)/4;
	eye_pose_matrix(row,col) = data_read;
      }
      else
      {
	if( i!=0 ) // attempt to read new matrix that doesn't exist
	{
	  std::runtime_error e("PoseCreator::fromFile::The number of elements in the eye pose file isn't valid. Nothing is read.");
	  throw e;
	}
	else
	{
	  calculation_finished = true;
	  break;
	}
      }
    }
    if( calculation_finished )
      break;
    
    geometry_msgs::Pose eye_pose;
    eye_pose.orientation = st_is::eigenToGeometry( Eigen::Quaterniond( eye_pose_matrix.topLeftCorner<3,3>() ) );
    eye_pose.position = st_is::eigenToGeometry( Eigen::Vector3d(eye_pose_matrix.rightCols(1)) );;
    eye_poses.push_back( eye_pose );
  }
  
  if( hand_poses.size()!=eye_poses.size() )
  {
    using namespace std;
    std::runtime_error e("PoseCreator::fromFile::The number of poses in the hand and eye pose files are not equal. Nothing is read.");
    throw e;
  }
  pose_pairs_.clear();
  
  for( unsigned int i=0; i<hand_poses.size(); i++ )
  {
    TransformationEstimator::PoseData new_pose_pair;
    new_pose_pair.hand_pose = hand_poses[i];
    new_pose_pair.eye_pose = eye_poses[i];
    
    pose_pairs_.push_back(new_pose_pair);
  }
}

Eigen::Matrix3d PoseCreator::rotationNoise( double _stddev )
{
  if( _stddev==0 )
    return Eigen::MatrixXd::Identity(3,3);
  
  std::random_device rd;
  std::mt19937 rng( rd() ); // mersenne twister random number generator with random seed
  std::normal_distribution<double> random_variable( 0, _stddev );
  
  
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd( random_variable(rng), Eigen::Vector3d::UnitX() )
  * Eigen::AngleAxisd( random_variable(rng), Eigen::Vector3d::UnitY() )
  * Eigen::AngleAxisd( random_variable(rng), Eigen::Vector3d::UnitZ() );
  
  return m;
}

Eigen::Vector3d PoseCreator::translationNoise( double _stddev )
{
  if( _stddev==0 )
    return Eigen::Vector3d(0,0,0);
  
  std::random_device rd;
  std::mt19937 rng( rd() ); // mersenne twister random number generator with random seed
  std::normal_distribution<double> random_variable( 0, _stddev );
  
  return Eigen::Vector3d( random_variable(rng), random_variable(rng), random_variable(rng) );
}