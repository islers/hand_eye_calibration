 
 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2014
*
*
* Class to estimate the transformation from a robot link (hand) to a camera 
* sensor frame (eye) [Hand-Eye-Calibration] using the method described in
* "Hand-Eye Calibration Using Dual Quaternions" by Konstantinos Daniilidis.
* It features a simple command line interface, the estimated transformations
* are printed to the command line as well.
* 
* subscribes:	- /hec/eye_position [geometry_msgs/Pose]: transformation grid->camera [rotation R_CG, position of origin of G in C
* 		- /hec/hand_position [geometry_msgs/Pose]: transformation hand->base  [rotation R_BH, position of origin of H in B
* 
* 
* Released under the GNU Lesser General Public License v3 (LGPLv3), see www.gnu.org
*
******************************************************************************/

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

typedef pair<geometry_msgs::Pose,geometry_msgs::Pose> calibPair;

class DualQuaternionTransformationEstimator
{
  public:
    DualQuaternionTransformationEstimator( ros::NodeHandle* _n );
    ~DualQuaternionTransformationEstimator();
    
    /** adds the next eye- and the next hand-position to the list of
     * pose pairs which will be used for estimation*/
    void addLastRetrievedPosePair();
    
    /** deletes the last added pose pair */
    void deleteLastAddedPosePair();
    
    /** calculates the transformation estimate */
    void calculateTransformation();
    
    /** returns the calculated transformation */
    geometry_msgs::Pose getHandToEye();
    
    /** returns the rotation matrix R_EH */
    Matrix3d rotH2E();
    
    /** returns the rotation matrix R_HE */
    Matrix3d rotE2H();
    
    /** returns the translation vector H_t_EH */
    Vector3d transH2E();
    
    /** returns the translation vector E_t_HE */
    Vector3d transE2H();
    
    /** returns the transformation matrix H_EH from hand to eye coordinates */
    Matrix<double,4,4> matrixH2E();
    
    /** returns the transformation matrix H_HE from eye to hand coordinates */
    Matrix<double,4,4> matrixE2H();
    
    /** clears all data to restart recording */
    void clearAll();

    void handListening( const geometry_msgs::PoseConstPtr& _newPose );
    void eyeListening( const geometry_msgs::PoseConstPtr& _newPose );
    
    /** returns the number of pose pairs added so far */
    int count();
    
    
    /** saves the hand and eye poses to a file, using the opencv storage functionality. Returns true if no problems occured */
    bool printToFile( string fileName_ );
    
    /** loads hand and eye poses from a file, saved using the printToFile method: both must be saved like a OpenCV Mat matrix with size 7xNumberOfPoses, where indices 0...3 represent the rotation quaternion and 4..6 the translation vector and the number of poses must be equal for both. The name of the hand poses must be "handPoses", the one of the eye poses "eyePoses". Returns true if succesful.
     * 
     * If destroyOldData_ is set to true, any previous hand-eye correspondences are dropped. If it is false, the correspondences loaded from file are added to the ones already stored.
    */
    bool loadFromFile( string fileName_, bool destroyOldData_=false );
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    ros::Subscriber handSubscriber_;
    ros::Subscriber eyeSubscriber_;
    
    vector< pair<geometry_msgs::Pose, geometry_msgs::Pose> > posePairs_; //pairs: hand,cam
    
    /** simple solver for the quadratic equation a*x² + bx + c = 0
     *  Returns false if the roots are imaginary, otherwhise the two roots are stored in _roots - twice
     *  the same value if only one root exists.
     */
    bool roots( double _aCoeff, double _bCoeff, double _cCoeff, pair<double,double>& _roots );
        
    bool transformationCalculated_;
    bool handRecorded_, eyeRecorded_;
    ros::Time recordedHandTimeStamp_, recordedEyeTimeStamp_;
    
    Quaterniond rot_EH_; // estimated rotation from hand to eye
    Vector3d H_trans_EH_; // estimated translation vector from hand to eye in hand coordinates
    
    Matrix3d crossProdMatrix( Vector3d _vec );
    Eigen::Vector3d geometryToEigen( const geometry_msgs::Point& _vec );
    Eigen::Quaterniond geometryToEigen( const geometry_msgs::Quaternion& _quat );
    pair<Eigen::Quaterniond,Eigen::Quaterniond> dualQuaternion( Eigen::Quaterniond _rot, Eigen::Vector3d _trans );
    
    geometry_msgs::Pose bufferedHand_, bufferedEye_;
        
    ros::NodeHandle* rosNode_;
};