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

#pragma once

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>

#include "hand_eye_calibration/HandPose.h"
#include "hand_eye_calibration/CameraPose.h"
#include "hand_eye_calibration/calibration_setup.h"

#include "utils/math.h"
#include "utils/eigen_utils.h"

/// abstract base class for transformation estimators
class TransformationEstimator
{  
public:
  class EstimationData; /// describes an estimation
  struct TransformationError;
  class EstimationMethod;
  struct PoseData; /// describes one measured hand-eye pose pair

    TransformationEstimator( ros::NodeHandle* _n );
    ~TransformationEstimator();
    
    /** Returns the name of the method used for calibration */
    virtual std::string estimationMethod();
    
    /** sets a new estimation method
     */
    void setEstimationMethod( boost::shared_ptr<EstimationMethod> _new_method );
    
    /** requests new hand and eye poses from the hec_eye_pose and hec_hand_pose services, adds this pair
     * The new poses are obtained through calls to the hec_eye_pose and hec_hand_pose services.
     * @return bool true if a new pose pair was successfully obtained and added
     */
    virtual bool addNewPosePair();
    
    
    /** requests new hand and eye poses from the hec_eye_pose and hec_hand_pose services, adds this pair and calculates a new transformation estimate if enough data is available
     * The new poses are obtained through calls to the hec_eye_pose and hec_hand_pose services.
     * @return bool true if a new pose pair was successfully obtained and added and a new estimation was calculated
     */
    virtual bool addAndEstimate();
    
    /** adds the next eye- and the next hand-position to the list of
     * pose pairs which will be used for estimation and calculates a new transformation estimate which is stored in the respective vector
     * Note: This works only if listening to topics has been activated. Since no calibration
     * pattern positions are pusblished on the topic this adds no data that can be used for
     * the reprojection error.
     */
    virtual void addLastRetrievedPosePair();
    
    /** deletes the last added pose pair */
    virtual void deleteLastAddedPosePair();
    
    /** add last deleted pose pair back to list */
    virtual void restoreLastDeletedPosePair();
    
    /** returns true if estimating is possible
     */
    bool estimationPossible();
    
    /** returns the last estimate, attempts to create one if none has been created yet
     * @throws std::runtime_error if the method is called but no estimation can be or has been done because not enough data is available
     */
    EstimationData estimate();
    
    /** creates a new estimation and adds it to the internal estimation list 
     * @throws std::runtime_error if the method is called but no estimation can be done because not enough data is available
     */
    void createAndAddNewEstimation();
    
    /** creates a new estimation based on all currently available data, including error estimates
     * @throws std::runtime_error if the method is called but no estimation can be done because not enough data is available
     */
    EstimationData getNewEstimation();
    
    
    /** deletes all estimations
     */
    void clearEstimations();
    
    /** deletes last estimation
     */
    void deleteLastEstimation();
    
    /** calculates a reprojection error measure for given estimate based on all data currently available (pattern_coordinate sets)
     * 
     * @param _estimation_data the estimation - error data is not overwritten
     * @param _reprojection_error to return the calculated reprojection error
     * @return true if calculating a reprojection error was possible
     * 
     */
    bool getReprojectionError( EstimationData& _estimation_data, st_is::StdError& _reprojection_error );
    
    /** calculates transformation error measures for given estimate based on all data currently available (pose pairs)
     * 
     * @param _estimation_data the estimation - error data is not overwritten
     * @return pair with euler_angle_rms_error(first) and relative_translation_error(second)
     * 
     */
    TransformationError getTransformationError( EstimationData& _estimation_data );
    
    /** returns the last hand pose
     * @return  (R_BH and B_t_BH)
     */
    st_is::CoordinateTransformation lastHandPose();
    
    /** returns the last eye pose
     * @return  R_EO and E_t_EO
     */
    st_is::CoordinateTransformation lastEyePose();
    
    /** calculates an estimate for the transformation from calibration pattern coordinate system to robot base coordinate system H_BO (an average using all pose measurements)
     * @param _estimation_data hand-eye-transformation to use
     * @throws range_error if no pose data is available
     */
    st_is::CoordinateTransformation getCalibrationPatternPoseEstimate( EstimationData& _estimation_data );
    
    /** returns the current calibration configuration */
    CalibrationSetup getCalibrationSetup();
    
    /** sets a new calibration configuration
     * @param _new_configuration new configuration to be set
     */
    void setCalibrationConfiguration( CalibrationSetup& _new_configuration );
    
    
    /** sets a new calibration configuration
     * @param _projection_matrix camera projection matrix
     * @param _calibration_pattern_world_coordinates calibration pattern in its own (3D) coordinate system
     */
    void setCalibrationConfiguration( Eigen::Matrix<double,3,4>& _projection_matrix, std::vector<geometry_msgs::Point>& _calibration_pattern_world_coordinates, unsigned int _image_height, unsigned int _image_width );
    
    /** attempts to load a new calibration configuration by calling the 'hec_eye_node_info' service
     * @return true if calling the service succeeded and a new configuration has been loaded
     */
    bool loadCalibrationConfigurationFromService();
    
    /** clears all data to restart recording (data is moved to trash bin) */
    virtual void clearAll();
    
    /** sets a new service wait time: the time services have to answer the request
     * @param _wait_time the new wait time in ms
     */
    virtual void setNewServiceWaitTime( unsigned int _wait_time );
    
    /** starts listening to pose topics */
    virtual void startListening();
    
    /** stops listening to pose topics */
    virtual void stopListening();

    void handListening( const geometry_msgs::PoseConstPtr& _newPose );
    void eyeListening( const geometry_msgs::PoseConstPtr& _newPose );
    
    /** returns the number of pose pairs added so far */
    virtual int count();
    
    
    /** saves the hand and eye poses and all estimates generated to a file, using the opencv storage functionality. Returns true if no problems occured, currently it does not save the images
     * saves:
     * 		- pose_pairs_
     * 		- calib_pattern_image_coordinates_
     * 		- transformation_estimates_t:
     * 		  rot_x | rot_y | rot_z | rot_w | trans_x | trans_y | trans_z | reprojection error mean | reprojection error std dev | euler angle error mean | euler angle error stddev | relative translation error mean | relative translation error stddev
     * 
     * Errors are set to -1000 if they haven't been calculated for an estimate
     */
    virtual bool printToFile( std::string fileName_ );
    
    /** loads hand and eye poses from a file, saved using the printToFile method: both must be saved like a OpenCV Mat matrix with size 7xNumberOfPoses, where indices 0...3 represent the rotation quaternion and 4..6 the translation vector and the number of poses must be equal for both. The name of the hand poses must be "handPoses", the one of the eye poses "eyePoses". Returns true if succesful.
     * 
     * If destroyOldData_ is set to true, any previous hand-eye correspondences are dropped. If it is false, the correspondences loaded from file are added to the ones already stored.
    */
    virtual bool loadFromFile( std::string fileName_, bool destroyOldData_=false );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    ros::Subscriber hand_subscriber_;
    ros::Subscriber eye_subscriber_;
    
    std::vector< boost::shared_ptr<EstimationMethod> > estimation_methods_; /// added estimation method - currently only the first in the vector is used, but using a vector in order to allow for extendence to a set of methods
    
    ros::Duration max_service_wait_time_; /// max time the pose services have to answer the request, default is 30 ms
    
    std::vector<PoseData> pose_data_; /// stores all poses and related data    
    std::vector<PoseData> del_pose_data_; /// trash bin for pose data
    
    Eigen::Quaterniond rot_EH_; /// current estimated rotation from hand to eye
    Eigen::Vector3d E_trans_EH_; /// current estimated position of H (hand) origin in E (eye) coordinates
    std::vector<EstimationData, Eigen::aligned_allocator<EstimationData> > transformation_estimates_; // holds different estimations along with their errors
    
    CalibrationSetup calibration_configuration_;
    
    void dumpTrash(); /// deletes all contente of the del_* vectors
    
    bool hand_recorded_, eye_recorded_;
    ros::Time recorded_hand_time_stamp_, recorded_eye_time_stamp_;
        
    geometry_msgs::Pose buffered_hand_, buffered_eye_;
        
    ros::NodeHandle* ros_node_;
    
    /** calculates a reprojection error measure for the data at time _i
     * @param _i time
     * @param _estimation_data estimate for which the error is calculated
     * @throws std::invalid_argument if no data is available at time _i
     * @throws std::range_error if _i out of range
     */
    double reprojectionErrorForTime( unsigned int _i, EstimationData& _estimation_data );
    
    /** calculates reprojection error measure for point _j at time _i by using an average reprojection error calculated with all known poses
     * @param _i time index
     * @param _j point index (index in calibration pattern vector)
     * @param _estimation_data estimate for which the error is calculated
     * @throws std::range_error if _j out of range
     */
    double reprojectionErrorForTimeAndPoint( unsigned int _i, unsigned int _j, EstimationData& _estimation_data  );
    
    /** calculates reprojection error measure for point _j at time _i given the camera pose
     * @param _i time index
     * @param _j point index (index in calibration pattern vector)
     * @param _camera_pose  The transformation from calibration pattern world coordinates (O) to eye coordinates (E): the rotation R_EO and the translation vector E_t_EO, Therefore, a transformation O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
     * @throws std::range_error if _j out of range
     */
    double reprojectionErrorForTimeAndPointGivenProjection( unsigned int _i, unsigned int _j, geometry_msgs::Pose& _camera_pose );
    
    /** calculates an estimate for the cam pose T_EO_i given the robot arm measurements at time i and an estimate for the hand-eye transformation
     * @param _i time index
     * @param _estimation_data hand-eye-transformation estimate
     */
    st_is::CoordinateTransformation camPoseEstimateForPoseData( PoseData& _pose, EstimationData& _estimation_data );
  
};

struct TransformationEstimator::PoseData
{
  geometry_msgs::Pose hand_pose; // R_BH and B_t_BH (B: robot base coordinate system, H: hand (robot end effector) coordinate system)
  geometry_msgs::Pose eye_pose; // R_EO and E_t_EO (E: eye(camera) coordinate system, O: calibration pattern coordinate system)
  
  std::vector<hand_eye_calibration::Point2D> calibration_pattern_coordinates;
  cv::Mat eye_image;
};