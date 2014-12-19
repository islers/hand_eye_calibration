 
#include "hand_eye_calibration/YoubotLinkPositionPublisher.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_link_position_publisher");
  ros::NodeHandle n;
  
  if( n.ok() )
  {
    YoubotLinkPositionPublisher youbotTransformations( &n,0,5 );
    youbotTransformations.run();
  }
  
  
  return 0;
}