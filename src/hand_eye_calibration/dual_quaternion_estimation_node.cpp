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


#include "hand_eye_calibration/dual_quaternion_transformation_estimator.h"
#include "hand_eye_calibration/estimation_data.h"
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual_quaternion_transformation_estimator");
  ros::NodeHandle n;
  
  TransformationEstimator estimator( &n );
  ROS_INFO("Successfully started dual quaternion hand-eye calibration node");
  boost::shared_ptr<TransformationEstimator> daniilidis_estimator( new TransformationEstimator(&n) );
  boost::shared_ptr<TransformationEstimator::EstimationMethod> daniilidis( new DaniilidisDualQuaternionEstimation() );
  estimator.setEstimationMethod(daniilidis);
  //estimator.startListening();
  estimator.loadCalibrationConfigurationFromService();
  
  while( n.ok() )
  {
    ros::spinOnce();
    
    cout<<endl<<"Currently "<<estimator.count()<<" hand-eye pose-correspondences have been added."<<endl;
    cout<<endl<<"If you are satisfied with the hand and eye poses currently published and want to add them to the estimation set, press the 'a' key. If you want to start the estimation procedure, press 'e'. To delete the last added pair, type 'd', to delete all sets and restart from scratch, type 'c'. To load data from file, type 'l', to store the data to a file, type 's'. 'q' quits the programm"<<endl;
    char user_input;
    cin >> user_input;
    
    switch(user_input)
    {
      case 'a': estimator.addNewPosePair();
		if( estimator.estimationPossible() )estimator.createAndAddNewEstimation(); // create an estimation for it
		break;
      case 'e': cout<<endl<<"The calculated transformation matrix from hand coordinates to eye coordinates is:"<<endl;
		
		if(estimator.estimationPossible())
		  cout<<estimator.getNewEstimation().matrixE2H(); // output T_HE
		else
		  cout<<"Couldn't calculate estimate since data available was insufficient.";
      
		break;
      case 'd': estimator.deleteLastAddedPosePair(); break;
      case 'c': estimator.clearAll(); break;
      case 'q': return 0;
      case 'l': {
		cout<<endl<<"Do you want to delete all previous data? (Y/n), Or type 'q' to go back to the main routine."<<endl;
		char delete_data_input;
		bool delete_data = false;
		cin >> delete_data_input;
		
		if( delete_data_input=='Y' ) delete_data = true;
		else if( delete_data_input=='q' ) break;
		
		cout<<endl<<"Please enter the name of the data file."<<endl;
		string load_file;
		cin >> load_file;
		if( estimator.loadFromFile( load_file, delete_data ) ) cout<<endl<<"Data successfully loaded from file.";
		else cout<<endl<<"Loading data from file failed.";
		break;
		}
      case 's':	{
		cout<<endl<<"Please enter the name of the file you want to store the current data sets to."<<endl;
		string save_file;
		cin >> save_file;
		if( estimator.printToFile( save_file ) ) cout<<endl<<"Data successfully saved to file."<<endl;
		else cout<<endl<<"Saving data to file failed."<<endl;
		break;
		}
      default: cout<<"Unknown entry. Try again."<<endl;
    };
    
  }
  
  
  return 0;
} 
