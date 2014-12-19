#include "hand_eye_calibration/YoubotLinkPositionPublisher.hpp"

YoubotLinkPositionPublisher::YoubotLinkPositionPublisher( ros::NodeHandle* _n, int _sourceId, int _targetId ):
tf2Listener_(tfCore_)
{
  rosNode_ = _n;
  
  posePublisher_ = rosNode_->advertise<geometry_msgs::Pose>("/hec/hand_position",10);
  
  linkNames_.push_back("arm_link_0"); // arm base which is rigidly fixed to the youbot base
  linkNames_.push_back("arm_link_1");
  linkNames_.push_back("arm_link_2");
  linkNames_.push_back("arm_link_3");
  linkNames_.push_back("arm_link_4");
  linkNames_.push_back("arm_link_5");
  
  if( _sourceId<0 ) _sourceId=0;
  else if ( _sourceId > linkNames_.size()-1 ) _sourceId = linkNames_.size()-1;
    
  if( _targetId<0 ) _targetId=0;
  else if ( _targetId > linkNames_.size()-1 ) _targetId = linkNames_.size()-1;
  
  sourceFrameId_ = _sourceId;
  targetFrameId_ = _targetId;
  
}


YoubotLinkPositionPublisher::~YoubotLinkPositionPublisher()
{
  
}


void YoubotLinkPositionPublisher::run()
{
  cout<<endl<<"YoubotLinkPositionPublisher:: starting transformation calculation from "<<linkNames_[sourceFrameId_]<<" to "<<linkNames_[targetFrameId_]<<"."<<endl;
  
  ros::Rate rate(30.0);
  while( rosNode_->ok() )
  {
    geometry_msgs::TransformStamped transformation;
    try
    { // use tf2 features to calculate the wanted transformation
      transformation = tfCore_.lookupTransform( linkNames_[sourceFrameId_], linkNames_[targetFrameId_], ros::Time(0) );
    }
    catch( tf2::TransformException& ex)
    {
      ROS_ERROR("YoubotLinkPositionPublisher::run:: %s", ex.what() );
      rate.sleep();
      continue;
    }
    
    geometry_msgs::Pose newPose;
    newPose.position.x = transformation.transform.translation.x;
    newPose.position.y = transformation.transform.translation.y;
    newPose.position.z = transformation.transform.translation.z;
    newPose.orientation = transformation.transform.rotation;
    
    posePublisher_.publish(newPose);
    
    rate.sleep();
  }
  
  
  return;
}