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


#include <string>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>

#include "hand_eye_calibration/autonomous_hand_eye_calibrator.h"
#include "hand_eye_calibration/estimation_data.h"
#include "utils/ros_eigen.h"

#include "hand_eye_calibration/CameraPose.h"
#include "hand_eye_calibration/CameraPoseInfo.h"
#include "hand_eye_calibration/HandPose.h"

AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n ):
  ros_node_(_n),
  position_initialized_(false),
  image_border_tolerance_(50)
{
  initializeJointSpace();
  
  std::string moveit_group_name;
  if( !ros_node_->getParam("moveit_group_name",moveit_group_name) )
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::No MoveIt! group name given. Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL_STREAM(error);
    ros::shutdown();
    return;
  }
  if( !ros_node_->getParam("robot_base_frame",robot_base_frame_) )
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::No robot base frame specified. Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL_STREAM(error);
    ros::shutdown();
    return;
  }
  if( !ros_node_->getParam("hand_frame",robot_hand_frame_) )
  {
    std::string error = "AutonomousHandEyeCalibrator::AutonomousHandEyeCalibrator( ros::NodeHandle* _n )::line "+std::to_string(__LINE__)+"::No robot hand frame specified. Check your auto_config.yaml file. Shutting down node.";
    ROS_FATAL_STREAM(error);
    ros::shutdown();
    return;
  }
  
  // initialize estimators
  boost::shared_ptr<TransformationEstimator> daniilidis_estimator( new TransformationEstimator(_n) );
  boost::shared_ptr<TransformationEstimator::EstimationMethod> daniilidis( new DaniilidisDualQuaternionEstimation() );
  daniilidis_estimator->setEstimationMethod(daniilidis);
  estimators_.push_back( daniilidis_estimator );
  
  scene_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>( new planning_scene_monitor::PlanningSceneMonitor("robot_description") );
  scene_->startStateMonitor();
  scene_->startSceneMonitor();
  scene_->startWorldGeometryMonitor();
  
  robot_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>( new moveit::planning_interface::MoveGroup(moveit_group_name) );
  
  eye_client_ = ros_node_->serviceClient<hand_eye_calibration::CameraPose>("hec_eye_pose");
  hand_client_ = ros_node_->serviceClient<hand_eye_calibration::HandPose>("hec_hand_pose");
  
  cameraPubNodeInfoAvailable(); // attempts to gather information
  
}

void AutonomousHandEyeCalibrator::addTransformationEstimator( boost::shared_ptr<TransformationEstimator> _new_estimator )
{
  estimators_.push_back( _new_estimator );
}

void AutonomousHandEyeCalibrator::addTransformationEstimationMethod( boost::shared_ptr<TransformationEstimator::EstimationMethod> _method )
{
  boost::shared_ptr<TransformationEstimator> new_estimator( new TransformationEstimator(ros_node_) );
  new_estimator->setEstimationMethod(_method);
  estimators_.push_back( new_estimator );
}

bool AutonomousHandEyeCalibrator::runSingleIteration()
{
  
  if( !position_initialized_ ) // first iteration, initialize position, check if all listed joints are available for actuation through the move group with the provided name
  {
    initializePosition();
  }
  else
  { // get to next position
    
    bool found_new_pos = calculateNextJointPosition();
    if( !found_new_pos )
      return false;
    //else
    //  setTargetToNewPosition();
    
    if( !planAndMove() ) // don't have to move for first position
      return true; // no calculation for current new pos, but still new positions available
  }
  
  // update all transformation estimators added
  BOOST_FOREACH( boost::shared_ptr<TransformationEstimator> estimator, estimators_ )
  {
    estimator->addAndEstimate();
  }
   
  
  return true;
}

double AutonomousHandEyeCalibrator::maxRelEstimateChange()
{
  return 1.0; /////////////////////////////////////////////////////// dummy
}

void AutonomousHandEyeCalibrator::setImageBorderTolerance( double _tolerance )
{
  image_border_tolerance_ = _tolerance;
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
    else
      joint_names_.push_back( jointNames[i] );
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

void AutonomousHandEyeCalibrator::initializePosition()
{
  std::vector<std::string> active_moveit_joints;
  active_moveit_joints = robot_->getActiveJoints(); // get activated joints in the constructed move group
  
  bool all_joints_available = true;
  // check if all joints that shall be actuated actually are activated joints in the MoveGroup
  for( unsigned int i=0; i<joint_names_.size(); i++ )
  {
    bool joint_available = false;
    for( unsigned int j=0; j<active_moveit_joints.size(); j++ )
    {
      if( joint_names_[i] == active_moveit_joints[j] )
      {
	joint_available = true;
	break;
      }
    }
    if( joint_available == false )
    {
      all_joints_available = false;
      break;
    }
  }
  
  if( !all_joints_available )
  {
    std::string error = "AutonomousHandEyeCalibrator::runSingleIteration()::Not all joints given in the auto_config.yaml file are available for actuation in the MoveIt group '" + robot_->getName() + "' provided. The dimensions to be activated are";
    
    for( unsigned int i=0; joint_names_.size(); i++ )
      error += " "+joint_names_[i];
    
    error += ". The actuated joints in the MoveGroup are";      
    for( unsigned int i=0; active_moveit_joints.size(); i++ )
      error += " '"+active_moveit_joints[i]+"'";
    error += ". No iteration will be carried out.";
    
    ROS_ERROR( "%s",error.c_str() );
    
    return;
  }
  else
  {
    // initialize position
    //scene_->getStateMonitor()->waitForCurrentState(10);
    
    planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
    
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    //current_robot_state->update();
    std::vector<double> current_state;
    current_robot_state.copyJointGroupPositions( robot_->getName(), current_state );
    //current_state = robot_->getCurrentJointValues();
    
    ROS_INFO("Initializing with current robot state.");
    
    for( unsigned int i=0; i<joint_names_.size(); i++ )
    {
      for( unsigned int j=0; j<active_moveit_joints.size(); j++ )
      {
	if( joint_names_[i] == active_moveit_joints[j] )
	{
	  joint_position_.getDimValue( joint_names_[i] ) = current_state[j];
	  std::cout<<std::endl<<joint_names_[i]<<": "<<current_state[j];
	}
      }
      
    }
    joint_position_.resolvingSplit();
    position_initialized_ = true;
  }
  return;
}

bool AutonomousHandEyeCalibrator::calculateNextJointPosition()
{  
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState state_to_check = current_scene->getCurrentState();
    
  do{
    if( joint_position_.reachedTop() )
      return false;
    
    joint_position_++;
    setRobotStateToCurrentJointPosition( state_to_check );
    
  }while( !isCollisionFree(current_scene, state_to_check) || !calibrationPatternExpectedVisible(state_to_check) );
  robot_->setJointValueTarget( state_to_check );
  return true;
}

bool AutonomousHandEyeCalibrator::isCollisionFree( planning_scene_monitor::LockedPlanningSceneRO& _scene, robot_state::RobotState& _robot )
{
  bool colliding = _scene->isStateColliding( _robot );
  
  if( colliding )
  {
    ROS_INFO("Skipping state that would lead to collision.");
    ros::Duration wait(1,0);
    wait.sleep();
  }
  
  return !colliding;
}

bool AutonomousHandEyeCalibrator::calibrationPatternExpectedVisible( robot_state::RobotState& _robot )
{
  if( !cameraPubNodeInfoAvailable() )
  {
    ROS_INFO("No camera calibration info available. Going on with blind iteration.");
    return true;
  }
  if( !estimators_.front()->estimationPossible() )
  {
    ROS_INFO("No hand-eye estimate available yet. Going on with blind iteration.");
    return true;
  }
  /* calculation of next camera pose t_EP
   * **********************************
   * space indices used:
   * - E: eye space
   * - P: calibration pattern space
   * - B: robot base space
   * - H: hand space
   * - R: space of last actuated robot link
   * - G: planning frame (model frame) of the moveit robot object
   */
  TransformationEstimator::EstimationData current_hec_estimate = estimators_.front()->estimate();
  st_is::CoordinateTransformation t_EH( current_hec_estimate.rot_EH(), current_hec_estimate.E_trans_EH() );
  
  st_is::CoordinateTransformation t_BP = estimators_.front()->getCalibrationPatternPoseEstimate(current_hec_estimate);
    
  Eigen::Matrix<double,4,4> m_HG = _robot.getFrameTransform( robot_hand_frame_ ).matrix();
  Eigen::Quaterniond quat_HG( m_HG.topLeftCorner<3,3>() );
  Eigen::Vector3d H_t_HG = m_HG.topRightCorner<3,1>();
  st_is::CoordinateTransformation t_HG( quat_HG, H_t_HG );
  
  Eigen::Matrix<double,4,4> m_BG = _robot.getFrameTransform( robot_base_frame_ ).matrix();
  Eigen::Quaterniond quat_BG( m_BG.topLeftCorner<3,3>() );
  Eigen::Vector3d B_t_BG = m_BG.topRightCorner<3,1>();
  st_is::CoordinateTransformation t_BG( quat_BG, B_t_BG );
  
  st_is::CoordinateTransformation t_GB = t_BG.inv();  
  st_is::CoordinateTransformation t_HB = t_HG*t_GB;
  st_is::CoordinateTransformation t_EB = t_EH*t_HB;
  
  st_is::CoordinateTransformation t_EP = t_EB*t_BP;
  geometry_msgs::Pose camera_pose;
  camera_pose.orientation = st_is::eigenToGeometry(t_EP.rotation);
  camera_pose.position = st_is::eigenToGeometry(t_EP.translation);
  
  // get projected calibration pattern points using the given camera pose
  std::vector<hand_eye_calibration::Point2D> projected_calibration_pattern = estimators_.front()->getCalibrationSetup().getProjectedCoordinates(camera_pose);
  
  // check if all projected points are inside the image frame, enlarged using the image border tolerance value
  double x_upper_bound = estimators_.front()->getCalibrationSetup().imageWidth() + image_border_tolerance_;
  double x_lower_bound = -image_border_tolerance_;
  double y_upper_bound = estimators_.front()->getCalibrationSetup().imageHeight() + image_border_tolerance_;
  double y_lower_bound = -image_border_tolerance_;
  
  BOOST_FOREACH( hand_eye_calibration::Point2D point, projected_calibration_pattern )
  {
    if( !(point.x<=x_upper_bound && point.x>=x_lower_bound && point.y<=y_upper_bound && point.y>=y_lower_bound) )
    {
      ROS_INFO("Skipping state where the calibration pattern is expected not to be fully visible.");
      return false;
    }
  }
  
  return true;
}

bool AutonomousHandEyeCalibrator::planAndMove()
{
  
  std::vector<double> target_state_position;
  
  std::vector<std::string> joint_names = robot_->getJoints();
  
  // move() and execute() never unblock thus this target reaching function is used
  robot_state::RobotState target_robot_state = robot_->getJointValueTarget();
  
  std::cout<<std::endl<<std::endl<<"Moving to new position:";
  
  for( unsigned int i = 0; i < joint_names.size() ; i++ )
  {
    std::cout<<std::endl<<joint_names[i]<<": "<<target_robot_state.getVariablePosition(joint_names[i]);
    target_state_position.push_back( target_robot_state.getVariablePosition(joint_names[i]) );
  }
  std::cout<<std::endl;
  
  
  planning_scene_monitor::LockedPlanningSceneRO current_scene( scene_ );
  robot_state::RobotState current_robot_state = current_scene->getCurrentState();
  robot_->setStartState(current_robot_state);
  
  double joint_tolerance = robot_->getGoalJointTolerance();
  double velocity_tolerance = 0.0001;
  
  
  ros::AsyncSpinner spinner(1);
  
  scene_->unlockSceneRead();
  
  spinner.start();
  // plan and execute a path to the target state
  bool success = robot_->move();
  spinner.stop();
  scene_->lockSceneRead();
  
  if( !success ) return false;
    
  ros::Duration wait_time(0,10000000);
  
  
  bool finished = false;
  
  // ensuring that the robot has assumed the commanded target state
  ros::Time start_time = ros::Time::now();
  ros::Duration max_wait_time(10.0); // if execution takes longer, it is considered unsuccessful
  
  do
  {
    scene_->unlockSceneRead();
    ros::spinOnce();
    scene_->lockSceneRead();
    
    current_robot_state = current_scene->getCurrentState();
        
    finished = true;
    for( unsigned int i = 0; i < joint_names.size(); i++ )
    {
      finished = finished && ( abs( current_robot_state.getVariablePosition(joint_names[i]) - target_state_position[i] ) <= joint_tolerance ) && ( abs( current_robot_state.getVariableVelocity(joint_names[i]) ) <= velocity_tolerance );
    }
    
    if( ros::Time::now() > (start_time+max_wait_time) )
    {
      robot_->stop(); // stop trajectory execuation if still active
      ROS_WARN_STREAM("Execution of a move was signaled to be successful but the joint states suggest otherwise. Waited for "<<max_wait_time.toSec()<<" seconds for the joints to assume the commanded position, aborting now, considering it unsuccessful and going on with the calculation.");
      return false;
    }
    
    if( !finished )
      wait_time.sleep();
    
  }while( !finished );
  
  return true;
}

void AutonomousHandEyeCalibrator::setTargetToNewPosition()
{
  for( unsigned int i=0; i<joint_names_.size(); i++ )
  {
    // enforce boundaries
    double new_value = joint_position_.getDimValue(joint_names_[i]);
    if( new_value > *(*joint_position_)[i].upperLimit() ) new_value = *(*joint_position_)[i].upperLimit();
    else if( new_value < *(*joint_position_)[i].lowerLimit() ) new_value = *(*joint_position_)[i].lowerLimit();
    
    robot_->setJointValueTarget( joint_names_[i], new_value );
  }
  
  return;
}

void AutonomousHandEyeCalibrator::setRobotStateToCurrentJointPosition( robot_state::RobotState& _robot )
{  
  std::vector<double> joint_configuration;
  for( unsigned int i = 0; i<joint_names_.size(); i++ )
  {
    //joint_configuration.push_back( joint_position_[i] );
    _robot.setVariablePosition( joint_names_[i], joint_position_[i] );
  }
  //_robot.setVariablePositions( joint_names_, joint_configuration );
}

bool AutonomousHandEyeCalibrator::cameraPubNodeInfoAvailable()
{
  if( estimators_.front()->getCalibrationSetup().isSetup() )
    return true;
  
  return estimators_.front()->loadCalibrationConfigurationFromService();
}