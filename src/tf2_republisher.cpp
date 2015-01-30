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

#include "hand_eye_calibration/tf2_republisher.h"

using namespace std;

TF2Republisher::TF2Republisher( ros::NodeHandle* _n ):
tf2Listener_(tfCore_)
{
  rosNode_ = _n;
  
  posePublisher_ = rosNode_->advertise<geometry_msgs::Pose>("/hec/hand_position",10);
  hand_position_server_ = rosNode_->advertiseService("hec_hand_pose", &TF2Republisher::serviceHandPoseRequest, this );
  
  if( !rosNode_->getParam("hand/base_link",base_link_) )
  {
    std::string error = "TF2Republisher::TF2Republisher::line "+std::to_string(__LINE__)+"::No base link name provided on 'hec/hand/base_link'. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  if( !rosNode_->getParam("hec/hand/end_link",end_link_) )
  {
    std::string error = "TF2Republisher::TF2Republisher::line "+std::to_string(__LINE__)+"::No end link name provided on 'hand/end_link'. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }  
}


TF2Republisher::~TF2Republisher()
{
  
}

void TF2Republisher::run()
{
  cout<<endl<<"TF2Republisher:: starting transformation calculation from "<<base_link_<<" to "<<end_link_<<"."<<endl;
  
  ros::Rate rate(30.0);
  while( rosNode_->ok() )
  {    
    geometry_msgs::Pose newPose;
    if( calculateTransformation(newPose) )
    {
      cout<<endl<<"Publishing new link position:"<<endl;
      cout<<endl<<"The translation vector is:";
      cout<<endl<<newPose.position<<endl<<endl;
      cout<<endl<<"The rotation vector is:"<<endl<<newPose.orientation<<endl;
      
      posePublisher_.publish(newPose);
    }
    
    rate.sleep();
  }
  
  
  return;
}

bool TF2Republisher::serviceHandPoseRequest( hand_eye_calibration::HandPose::Request& _req, hand_eye_calibration::HandPose::Response& _res )
{
  ros::Time request_time = ros::Time::now();
  ros::Duration max_wait_time = _req.request.max_wait_time.data;
  
  ros::Time time_limit = request_time+max_wait_time;
  
  _res.description.stamp = request_time;
  _res.description.request_id = _req.request.request_id;
  
  geometry_msgs::Pose newPose;
  ros::Rate rate(5.0);
  while( ros::Time::now() <= time_limit )
  {
    if( calculateTransformation(newPose) )
    {
      _res.description.pose = newPose;
      _res.description.pose_found = true;
      return true;
    }
    ROS_INFO("TF2Republisher::serviceHandPoseRequest::called, waiting for new pose");
    rate.sleep();
    ros::spinOnce();
  }
  
  _res.description.pose_found = false;
  
  return true;
}

bool TF2Republisher::calculateTransformation( geometry_msgs::Pose& _hand_pose )
{
  geometry_msgs::TransformStamped transformation;
  try
  { /* use tf2 features to calculate the wanted transformation
    This gives the pose of the targetFrame (end effector) in the source's (arm base) coordinates. */
    transformation = tfCore_.lookupTransform( base_link_, end_link_, ros::Time(0) );
  }
  catch( tf2::TransformException& ex)
  {
    ROS_ERROR("TF2Republisher::run:: %s", ex.what() );
    return false;
  }
  
  _hand_pose.position.x = transformation.transform.translation.x;
  _hand_pose.position.y = transformation.transform.translation.y;
  _hand_pose.position.z = transformation.transform.translation.z;
  _hand_pose.orientation = transformation.transform.rotation;
  
  return true;
}