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
#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "hand_eye_calibration/CameraPose.h"
#include "hand_eye_calibration/CameraPoseInfo.h"
#include "hand_eye_calibration/HandPose.h"

AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n ):
  ros_node_(_n),
  daniilidis_estimator_(_n)
{
  initializeJointSpace();
  
  std::string moveit_group_name;
  if( !ros_node_->getParam("moveit_group_name",moveit_group_name) )
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::No MoveIt! group name given. Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  
  robot_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>( new moveit::planning_interface::MoveGroup(moveit_group_name) );
  robot_->startStateMonitor();
  
  eye_client_ = ros_node_->serviceClient<hand_eye_calibration::CameraPose>("hand_eye_eye_pose");
  hand_client_ = ros_node_->serviceClient<hand_eye_calibration::HandPose>("hand_eye_hand_pose");
  
  hand_eye_calibration::CameraPoseInfo cam_info;
  if( !ros::service::call("hand_eye_eye_node_info",cam_info) )
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::Could not contact service 'hand_eye_eye_node_info'. Shutting down.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  
}

bool AutonomousHandEyeCalibrator::runSingleIteration()
{  
  std::vector<double> current_state;
  current_state = robot_->getCurrentJointValues();
  for( unsigned int i=0; i<current_state.size(); i++ )
  {
    std::cout<<std::endl<<current_state[i];
  }
  std::cout<<std::endl;
  current_state[1] = 1.15;
  current_state[2] = -2.6;
  current_state[3] = 1.88;
  current_state[4] = 2.9;
  
  robot_->setJointValueTarget( current_state );
  
  planAndMove();
  
  
  return false; /////////////////////////////////////////////////////// dummy
}

double AutonomousHandEyeCalibrator::maxRelEstimateChange()
{
  return 1.0; /////////////////////////////////////////////////////// dummy
}

bool AutonomousHandEyeCalibrator::printToFile( std::string _filename )
{
  return false;  /////////////////////////////////////////////////////// dummy
}

void AutonomousHandEyeCalibrator::initializeJointSpace()
{
  // loads the names of the joints that will be iterated
  XmlRpc::XmlRpcValue jointNames;
  if (!ros_node_->getParam("joints", jointNames))
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::No joints given. (namespace: "+ros_node_->getNamespace()+"). Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  if (jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::Malformed joint specification.  (namespace: "+ros_node_->getNamespace()+"). Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  
  for (unsigned int i = 0; i < static_cast<unsigned int> (jointNames.size()); ++i)
  {
    XmlRpc::XmlRpcValue &name = jointNames[i];
      if (name.getType() != XmlRpc::XmlRpcValue::TypeString)
      {
	std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::Array of joint names should contain only strings.  (namespace: "+ros_node_->getNamespace()+"). Check your auto_config.yaml file. Shutting down node.";
	ROS_FATAL("%s",error.c_str());
	ros::shutdown();
	return;
      }
  }  
  
  for (unsigned int i = 0; i < static_cast<unsigned int> (jointNames.size()); ++i)
  {
    initializeJointConfiguration( jointNames[i] );
  }
  
  return;
}

void AutonomousHandEyeCalibrator::initializeJointConfiguration( std::string name_ )
{
  double lower_limit, upper_limit, step_size;
  
  if( !ros_node_->getParam("joint_space/"+(std::string)name_+"/lower_limit",lower_limit) )
  {
    std::string error = "AutonomousHandEyeCalibrator::initializeJointConfiguration( std::string name_ )::line "+std::to_string(__LINE__)+"::Could not find proper lower_limit configuration for joint "+name_+". Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  if( !ros_node_->getParam("joint_space/"+(std::string)name_+"/upper_limit",upper_limit) )
  {
    std::string error = "AutonomousHandEyeCalibrator::initializeJointConfiguration( std::string name_ )::line "+std::to_string(__LINE__)+"::Could not find proper upper_limit configuration for joint "+name_+". Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  if( !ros_node_->getParam("joint_space/"+(std::string)name_+"/step_size",step_size) )
  {
    std::string error = "AutonomousHandEyeCalibrator::initializeJointConfiguration( std::string name_ )::line "+std::to_string(__LINE__)+"::Could not find proper step_size configuration for joint "+name_+". Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL("%s",error.c_str());
    ros::shutdown();
    return;
  }
  
  st_is::NumericIterator<double> lower_end( lower_limit, step_size, lower_limit );
  st_is::NumericIterator<double> upper_end( lower_limit, step_size, upper_limit );
  
  st_is::IteratorBouncer< st_is::NumericIterator<double> > new_joint_dimension( lower_end, lower_end, upper_end );
  
  joint_position_.addDim( &new_joint_dimension, name_ );
  
  return;
}

void AutonomousHandEyeCalibrator::planAndMove()
{
  
  std::vector<double> target_state_position;
  
  std::vector<std::string> joint_names = robot_->getJoints();
  
  robot_state::RobotStatePtr current_robot_state;
  
  // move() and execute() never unblock thus this target reaching function is used
  robot_state::RobotState target_robot_state = robot_->getJointValueTarget();
  
  for( unsigned int i = 0; i < joint_names.size() ; i++ )
  {
    target_state_position.push_back( target_robot_state.getVariablePosition(joint_names[i]) );
  }
  
  double joint_tolerance = robot_->getGoalJointTolerance();
  double velocity_tolerance = 0.0001;
    
  
  // plan and execute a path to the target state
  moveit::planning_interface::MoveItErrorCode result = robot_->asyncMove();
     
  ros::Duration wait_time(0,10000000);
  
  bool succeeded = false;
  // check whether the robot has assumed the commanded target state yet or not
  while( !succeeded )
  {
    succeeded = true;
    current_robot_state = robot_->getCurrentState();
        
    for( unsigned int i = 0; i < joint_names.size(); i++ )
    {
      succeeded = succeeded && ( abs( current_robot_state->getVariablePosition(joint_names[i]) - target_state_position[i] ) <= joint_tolerance ) && ( abs( current_robot_state->getVariableVelocity(joint_names[i]) ) <= velocity_tolerance );
    }
    
    ros::spinOnce();
    wait_time.sleep();
  }
  return;
}