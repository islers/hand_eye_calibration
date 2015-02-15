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


/** Provides services with artificial hand-eye poses based on given or random hand-eye-transformations,
 * @todo (currently only the poses without pattern data are provided)
 */


#include "ros/ros.h"

#include <random>
#include <boost/foreach.hpp>

#include "utils/eigen_utils.h"
#include "hand_eye_calibration/estimation_data.h"
#include "hand_eye_calibration/pose_creator.h"

PoseCreator g_artificial_poses; // stores the currently created poses and creates new ones
std::deque< std::pair<ros::Time,int> > g_request_mapping; // maps "request time stamps" to "pose data indices"
int g_highest_used_pose_index; // stores the highest already provided pose index since the last pose calculation
int g_max_number_of_simultaneous_pose_requests = 10; // guaranteed number of possible simultaneous pose requests

TransformationEstimator::PoseData getPose( ros::Time _time_stamp )
{
  typedef std::pair<ros::Time,int> TimeIntPair;
  BOOST_FOREACH( TimeIntPair mapping, g_request_mapping )
  {
    if( mapping.first==_time_stamp )
      return g_artificial_poses.posePairs()[mapping.second];
  }
  // no pos assigned to that time_stamp ... assigne new pose
  // if necessary create new poses
  if( g_highest_used_pose_index==g_artificial_poses.posePairs().size()-1 )
  {
    // save possibly still used data
    std::vector<TransformationEstimator::PoseData> possibly_valid_poses;
    for( unsigned int i=0; i<g_request_mapping.size(); i++ )
    {
      possibly_valid_poses.push_back( g_artificial_poses.posePairs()[g_request_mapping[i].second] );
      g_request_mapping[i].second = i;
    }
    
    g_artificial_poses.calcPosePairs( 100 );
    
    for( unsigned int i=0; i<possibly_valid_poses.size(); i++ )
    {
      g_artificial_poses.posePairs()[i] = possibly_valid_poses[i];
    }
    g_highest_used_pose_index = possibly_valid_poses.size()-1;
  }
  g_highest_used_pose_index++;
  
  if( g_request_mapping.size()>=g_max_number_of_simultaneous_pose_requests )
    g_request_mapping.pop_front();
  g_request_mapping.push_back( std::pair<ros::Time,int>(_time_stamp,g_highest_used_pose_index) );
  
  return g_artificial_poses.posePairs()[g_highest_used_pose_index];
}

bool eyePositionService( hand_eye_calibration::CameraPose::Request& _req, hand_eye_calibration::CameraPose::Response& _res )
{
  _res.description.stamp = ros::Time::now();
  _res.description.request_stamp = _req.request.request_stamp;
  _res.description.pose_found = true;
  _res.description.pose = getPose( _req.request.request_stamp ).eye_pose;
    
  return true;
}

bool handPositionService(  hand_eye_calibration::HandPose::Request& _req, hand_eye_calibration::HandPose::Response& _res )
{
  _res.description.stamp = ros::Time::now();
  _res.description.request_stamp = _req.request.request_stamp;
  _res.description.pose_found = true;
  _res.description.pose = getPose( _req.request.request_stamp ).hand_pose;
    
  return true;
}



int main(int argc, char **argv)
{
  using namespace Eigen;
  using namespace st_is;
  
  ros::init(argc, argv, "fake_pose_server");
  ros::NodeHandle n("fake_pose_server");
  
  // initialize base poses (randomly first, but load from parameter if possible)
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<double> uniform_rv(-2*M_PI,2*M_PI);
  
  st_is::CoordinateTransformation t_RW, t_HE;
  
  Eigen::Vector3d rot_rw_axis( uniform_rv(rng),uniform_rv(rng),uniform_rv(rng) );
  rot_rw_axis.normalize();
  double rot_rw_angle = uniform_rv(rng);
  Eigen::Quaterniond rot_rw( Eigen::AngleAxisd(rot_rw_angle,rot_rw_axis) );
  
  if( !n.getParam("/hec/fake_pose/world_to_robot/rotation/x",t_RW.rotation.x() ) || 
      !n.getParam("/hec/fake_pose/world_to_robot/rotation/y",t_RW.rotation.y() ) ||
      !n.getParam("/hec/fake_pose/world_to_robot/rotation/z",t_RW.rotation.z() ) ||
      !n.getParam("/hec/fake_pose/world_to_robot/rotation/w",t_RW.rotation.w() ) )
  {
    ROS_INFO("One or more of '/hec/fake_pose/world_to_robot/rotation/*' parameters missing, using random rotation.");
    t_RW.rotation = rot_rw;
  }
  
  if( !n.getParam("/hec/fake_pose/world_to_robot/translation/x",t_RW.translation(0) ))
  {
    ROS_INFO("hec/fake_pose/world_to_robot/translation/x missing - using random value");
    t_RW.translation(0) = uniform_rv(rng);
  }
  if( !n.getParam("/hec/fake_pose/world_to_robot/translation/y",t_RW.translation(1) ))
  {
    ROS_INFO("hec/fake_pose/world_to_robot/translation/y missing - using random value");
    t_RW.translation(1) = uniform_rv(rng);
  }
  if( !n.getParam("/hec/fake_pose/world_to_robot/translation/z",t_RW.translation(2) ))
  {
    ROS_INFO("hec/fake_pose/world_to_robot/translation/z missing - using random value");
    t_RW.translation(2) = uniform_rv(rng);
  }
  
  Eigen::Vector3d rot_he_axis( uniform_rv(rng),uniform_rv(rng),uniform_rv(rng) );
  rot_he_axis.normalize();
  double rot_he_angle = uniform_rv(rng);
  Eigen::Quaterniond rot_he( Eigen::AngleAxisd(rot_he_angle,rot_he_axis) );
  
  if( !n.getParam("/hec/fake_pose/eye_to_hand/rotation/x",t_HE.rotation.x() ) || 
      !n.getParam("/hec/fake_pose/eye_to_hand/rotation/y",t_HE.rotation.y() ) || 
      !n.getParam("/hec/fake_pose/eye_to_hand/rotation/z",t_HE.rotation.z() ) || 
      !n.getParam("/hec/fake_pose/eye_to_hand/rotation/w",t_HE.rotation.w() ) )
  {
    ROS_INFO("One or more of '/hec/fake_pose/eye_to_hand/rotation/*' parameters missing, using random rotation.");
    t_HE.rotation = rot_he;
  }
  
  if( !n.getParam("/hec/fake_pose/eye_to_hand/translation/x",t_HE.translation(0) ))
  {
    ROS_INFO("hec/fake_pose/eye_to_hand/translation/x missing - using random value");
    t_HE.translation(0) = uniform_rv(rng);
  }
  if( !n.getParam("/hec/fake_pose/eye_to_hand/translation/y",t_HE.translation(1) ))
  {
    ROS_INFO("hec/fake_pose/eye_to_hand/translation/y missing - using random value");
    t_HE.translation(1) = uniform_rv(rng);
  }
  if( !n.getParam("/hec/fake_pose/eye_to_hand/translation/z",t_HE.translation(2) ))
  {
    ROS_INFO("hec/fake_pose/eye_to_hand/translation/z missing - using random value");
    t_HE.translation(2) = uniform_rv(rng);
  }
  
  double trans_noise=0, rot_noise=0;
  n.getParam("/hec/fake_pose/noise/rotation",rot_noise );
  n.getParam("/hec/fake_pose/noise/translation",trans_noise );
  
  
  g_artificial_poses = PoseCreator( t_RW.inv(), t_HE );
  g_artificial_poses.addNoise( rot_noise, trans_noise );
  
  
  g_artificial_poses.calcPosePairs( 100 );
  g_highest_used_pose_index = 0;
  
  ros::ServiceServer eye_position_server = n.advertiseService("/hec/eye_pose", &eyePositionService );
  ros::ServiceServer hand_position_server = n.advertiseService("/hec/hand_pose", &handPositionService );
  
  std::cout<<std::endl;
  ROS_INFO("fake pose server started");
  
  Eigen::Matrix<double,4,4> T_HE;
  g_artificial_poses.getHandEyeTransform(T_HE);
  ROS_INFO_STREAM("hand-eye transformation T_HE used for creating poses is:"<<std::endl<<std::endl<<T_HE );
  ROS_INFO_STREAM("hand-eye transformation T_EH used for creating poses is:"<<std::endl<<std::endl<<T_HE.inverse() );
  
  ros::spin();
    
  
  return 0;
} 
