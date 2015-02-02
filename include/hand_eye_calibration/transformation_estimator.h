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

/// abstract base class for transformation estimators
class TransformationEstimator
{
  class EstimationData; /// describes an estimation
  struct PoseData; /// describes one measured hand-eye pose pair
  
  public:
    TransformationEstimator( ros::NodeHandle* _n );
    ~TransformationEstimator();
    
    /** requests new hand and eye poses from the hec_eye_pose and hec_hand_pose services, adds this pair and calculates a new transformation estimate
     * The new poses are obtained through calls to the hec_eye_pose and hec_hand_pose services.
     * @return bool true if a new pose pair was successfully obtained and added
     */
    virtual bool addNewPosePair();
    
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
    
    /** calculates the transformation estimate */
    virtual void calculateTransformation(bool _suppressWarnings=false )=0;
    
    /** returns true if estimating is possible
     */
    bool estimationPossible();
    
    /** creates a new estimation and adds it to the internal estimation list 
     * @throws std::runtime_error if the method is called but no estimation can be done because not enough data is available
     */
    void createAndAddNewEstimation();
    
    
    /** deletes all estimations
     */
    void clearEstimations();
    
    /** deletes last estimation
     */
    void deleteLastEstimation();
    
    /** creates a new estimation based on all currently available data, including error estimates
     * @throws std::runtime_error if the method is called but no estimation can be done because not enough data is available
     */
    EstimationData getNewEstimation();
    
    /** calculates a reprojection error measure for given estimate based on all data currently available (pattern_coordinate sets)
     * 
     * @param _estimation_data the estimation - error data is not overwritten
     * @param _reprojection_error to return the calculated reprojection error
     * @return true if calculating a reprojection error was possible
     * 
     */
    bool getReprojectionError( EstimationData const& _estimation_data, double& _reprojection_error );
    
    /** calculates transformation error measures for given estimate based on all data currently available (pose pairs)
     * 
     * @param _estimation_data the estimation - error data is not overwritten
     * @return pair with euler_angle_rms_error(first) and relative_translation_error(second)
     * 
     */
    std::pair<double,double> getTransformationError( EstimationData const& _estimation_data );
    
    /** returns the calculated transformation */
    virtual geometry_msgs::Pose getHandToEye();
    
    /** returns the rotation matrix R_EH */
    virtual Eigen::Matrix3d rotH2E();
    
    /** returns the rotation matrix R_HE */
    virtual Eigen::Matrix3d rotE2H();
    
    /** returns the translation vector E_t_EH (position of H in E)*/
    virtual Eigen::Vector3d transH2E();
    
    /** returns the translation vector H_t_HE (position of E in H)*/
    virtual Eigen::Vector3d transE2H();
    
    /** returns the transformation matrix H_EH from hand to eye coordinates */
    virtual Eigen::Matrix<double,4,4> matrixH2E();
    
    /** returns the transformation matrix H_HE from eye to hand coordinates */
    virtual Eigen::Matrix<double,4,4> matrixE2H();
    
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
     * 		- transformation_estimates_t
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
    
    ros::Duration max_service_wait_time_; /// max time the pose services have to answer the request, default is 30 ms
    
    std::vector<PoseData> pose_data_; /// stores all poses and related data    
    std::vector<PoseData> del_pose_data_; /// trash bin for pose data
    
    Eigen::Quaterniond rot_EH_; /// current estimated rotation from hand to eye
    Eigen::Vector3d E_trans_EH_; /// current estimated position of H (hand) origin in E (eye) coordinates
    std::vector<EstimationData> transformation_estimates_; // holds different estimations along with their errors, updated when new data is added
    
    CalibrationSetup calibration_configuration_;
    
    void dumpTrash(); /// deletes all contente of the del_* vectors
            
    bool transformation_calculated_;
    bool hand_recorded_, eye_recorded_;
    ros::Time recorded_hand_time_stamp_, recorded_eye_time_stamp_;
        
    geometry_msgs::Pose buffered_hand_, buffered_eye_;
        
    ros::NodeHandle* ros_node_;
      
  
};

class TransformationEstimator::EstimationData
{
  public:
    EstimationData();
    
    // hand-eye estimation
    Eigen::Quaterniond rot_EH;
    Eigen::Vector3d E_trans_EH;
    
    void setReprojectionError( double _error );
    void setTransformationErrors( double _euler_angle_rms_error, double _relative_translation_error );
    void setTransformationErrors( std::pair<double,double> _errors ); // pair<euler_angle_rms_error,relative_translation_error>
    
    /// returns true if a reprojection_error is available and writes the value to the argument
    bool reprojectionError( double& _reprojection_error );
    /// returns true if a euler_angle_rms_error is available and writes the value to the argument
    bool eulerAngleRmsError( double& _euler_angle_rms_error );
    /// returns true if a relative_translation_error is available and writes the value to the argument
    bool relativeTranslationError( double& _relative_translation_error );
    
  private:
    // error measures for the estimation
    bool has_reprojection_error_;
    double reprojection_error_;
    bool has_transformation_error_;
    double euler_angle_rms_error_;
    double relative_translation_error_;
};

struct TransformationEstimator::PoseData
{
  geometry_msgs::Pose hand_pose;
  geometry_msgs::Pose eye_pose;
  
  std::vector<hand_eye_calibration::Point2D> calibration_pattern_coordinates;
  cv::Mat eye_image;
};