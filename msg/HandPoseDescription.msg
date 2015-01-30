#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp

#Id for answer to specific requests
time request_stamp

#Whether or not a pose was found at the time of a request
bool pose_found

#transformation hand->base  [rotation R_BH, position of origin of H in B]
geometry_msgs/Pose pose