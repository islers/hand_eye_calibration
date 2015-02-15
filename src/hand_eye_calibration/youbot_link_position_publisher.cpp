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

#include "hand_eye_calibration/youbot_link_position_publisher.h"

using namespace std;

YoubotLinkPositionPublisher::YoubotLinkPositionPublisher( ros::NodeHandle* _n, int _sourceId, int _targetId ):
tf2_listener_(tf_core_)
{
  ros_node_ = _n;
  
  pose_publisher_ = ros_node_->advertise<geometry_msgs::Pose>("/hec/hand_position",10);
  hand_position_server_ = ros_node_->advertiseService("/hec/hand_pose", &YoubotLinkPositionPublisher::serviceHandPoseRequest, this );
  
  link_names_.push_back("arm_link_0"); // arm base which is rigidly fixed to the youbot base
  link_names_.push_back("arm_link_1");
  link_names_.push_back("arm_link_2");
  link_names_.push_back("arm_link_3");
  link_names_.push_back("arm_link_4");
  link_names_.push_back("arm_link_5");
  
  if( _sourceId<0 ) _sourceId=0;
  else if ( _sourceId > link_names_.size()-1 ) _sourceId = link_names_.size()-1;
    
  if( _targetId<0 ) _targetId=0;
  else if ( _targetId > link_names_.size()-1 ) _targetId = link_names_.size()-1;
  
  source_frame_id_ = _sourceId;
  target_frame_id_ = _targetId;
  
}


YoubotLinkPositionPublisher::~YoubotLinkPositionPublisher()
{
  
}


void YoubotLinkPositionPublisher::run()
{
  cout<<endl<<"YoubotLinkPositionPublisher:: starting transformation calculation from "<<link_names_[source_frame_id_]<<" to "<<link_names_[target_frame_id_]<<"."<<endl;
  
  ros::Rate rate(30.0);
  while( ros_node_->ok() )
  {    
    geometry_msgs::Pose newPose;
    if( calculateTransformation(newPose) )
    {
      cout<<endl<<"Publishing new link position:"<<endl;
      cout<<endl<<"The translation vector is:";
      cout<<endl<<newPose.position<<endl<<endl;
      cout<<endl<<"The rotation vector is:"<<endl<<newPose.orientation<<endl;
      
      pose_publisher_.publish(newPose);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  
  return;
}

bool YoubotLinkPositionPublisher::serviceHandPoseRequest( hand_eye_calibration::HandPose::Request& _req, hand_eye_calibration::HandPose::Response& _res )
{
  ros::Time request_time = ros::Time::now();
  ros::Duration max_wait_time = _req.request.max_wait_time;
  
  ros::Time time_limit = request_time+max_wait_time;
  
  _res.description.stamp = request_time;
  _res.description.request_stamp = _req.request.request_stamp;
  
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
    ROS_INFO("YoubotLinkPositionPublisher::serviceHandPoseRequest::called, waiting for new pose");
    rate.sleep();
    ros::spinOnce();
  }
  
  _res.description.pose_found = false;
  
  return true;
}

bool YoubotLinkPositionPublisher::calculateTransformation( geometry_msgs::Pose& _hand_pose )
{
  geometry_msgs::TransformStamped transformation;
  try
  { /* use tf2 features to calculate the wanted transformation
    This gives the pose of the targetFrame (end effector) in the source's (arm base) coordinates. */
    transformation = tf_core_.lookupTransform( link_names_[source_frame_id_], link_names_[target_frame_id_], ros::Time(0) );
  }
  catch( tf2::TransformException& ex)
  {
    ROS_ERROR("YoubotLinkPositionPublisher::run:: %s", ex.what() );
    return false;
  }
  
  _hand_pose.position.x = transformation.transform.translation.x;
  _hand_pose.position.y = transformation.transform.translation.y;
  _hand_pose.position.z = transformation.transform.translation.z;
  _hand_pose.orientation = transformation.transform.rotation;
  
  return true;
}