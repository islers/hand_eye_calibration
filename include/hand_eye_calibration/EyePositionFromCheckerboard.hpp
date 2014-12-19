 /******************************************************************************
*
* Author:
* Stefan Isler, islerstefan@bluewin.ch, ETH Zürich 2014
*
*
* This class estimates the camera position from a rectified camera image with
* a checkerboard for a calibrated camera using OpenCV. The image with the
* detected checkerboard pattern is published to provide a feedback for
* calibration purposes.
* 
* subscribes:	- /camera/image_rect [sensor_msgs/Image]
* 
* publishes:	- /hec/eye_position [geometry_msgs/Pose]: The transformation from
* 		  checkerboard coordinates (O) to camera(eye) coordinates (E): the rotation
* 		  R_EO represented by a quaternion and the translation vector E_t_EO,
* 		  that is the coordinates of the origin of O in E coordinates.
* 		  Therefore, a transformation O->E would be carried out through:
* 		  x_E = R_EO*x_O + E_t_EO
* 
* Data needed on the parameter server (use either the command line or a yaml file):
* 		- /camera/fx (intrinsic camera matrix data)
* 		- /camera/fy (")
* 		- /camera/cx (")
* 		- /camera/cy (")
* 		- /hec/checkerboard/squares_per_column (nr of internal chkrbrd edges per column)
* 		- /hec/checkerboard/squares_per_row (nr of internal chckrbrd edges per row
* 		- /hec/checkerboard/square_size (the size of checkerboard fields in [m])
* 
* 
* Released under the GNU Lesser General Public License v3 (LGPLv3), see www.gnu.org
*
******************************************************************************/

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

class EyePositionFromCheckerboard
{
  public:
    EyePositionFromCheckerboard( ros::NodeHandle* _n );
    ~EyePositionFromCheckerboard();
    
    /** starts the listening */
    void run();

    void imageLoader(  const sensor_msgs::ImageConstPtr& _newImage );
  private:
    ros::Publisher posePublisher_;
    ros::Subscriber cameraStream_;
    
    bool newImageLoaded_; // whether a new image has been loaded
    cv::Mat currentImage_;
    cv::Mat cameraMatrix_;
    cv::Size patternSize_;
    vector<cv::Point3f> objectPointCoordinates_; // coordinates of to-be-detected checkerboard points in checkerboard frame
    
    bool initSuccess_; // indicates successful initialization of all parameters
    
    bool init(); //initializes (loads) parameters from parameter server
        
    ros::NodeHandle* rosNode_;
}; 
