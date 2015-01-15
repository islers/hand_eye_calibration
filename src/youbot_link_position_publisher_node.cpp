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

 
#include "hand_eye_calibration/youbot_link_position_publisher.h"
#include "utils/numeric_iterator.h"
#include "utils/iterator_bouncer.h"
#include "utils/multi_dimensional_space_iterator.h"

using namespace std;

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