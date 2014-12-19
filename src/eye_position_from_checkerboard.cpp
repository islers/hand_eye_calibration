 
#include "hand_eye_calibration/EyePositionFromCheckerboard.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eye_position_from_checkerboard");
  ros::NodeHandle n;
  
  if( n.ok() )
  {
    EyePositionFromCheckerboard eyePositionCalculator( &n );
    eyePositionCalculator.run();
  }
  
  
  return 0;
} 
