#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp

#Id for answer to specific requests
uint32 request_id

#Whether or not a pose was found at the time of a request
bool pose_found

#The transformation from checkerboard coordinates (O) to camera(eye) coordinates (E):
#the rotation R_EO represented by a quaternion and the translation vector E_t_EO, 
#that is the coordinates of the origin of O in E coordinates. Therefore, a transformation
#O->E would be carried out through: x_E = R_EO*x_O + E_t_EO
geometry_msgs/Pose pose

#The image for which the pose was extracted, filled if a new image was retrieved, even if no pose was found
sensor_msgs/Image image

#Image coordinates [u,v] of the detected pattern
hand_eye_calibration/Point2D[] point_coordinates