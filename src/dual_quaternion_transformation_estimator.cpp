#include "hand_eye_calibration/DualQuaternionTransformationEstimator.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual_quaternion_transformation_estimator");
  ros::NodeHandle n;
  
  DualQuaternionTransformationEstimator estimator( &n );
  
  ROS_INFO("Successfully started dual quaternion hand-eye calibration node");
  
  while( n.ok() )
  {
    ros::spinOnce();
    
    cout<<endl<<"Currently "<<estimator.count()<<" hand-eye pose-correspondences have been added."<<endl;
    cout<<endl<<"If you are satisfied with the hand and eye poses currently published and want to add them to the estimation set, press the 'a' key. If you want to start the estimation procedure, press 'e'. To delete the last added pair, type 'd', to delete all sets and restart from scratch, type 'c'. To load data from file, type 'l', to store the data to a file, type 's'. 'q' quits the programm"<<endl;
    char userInput;
    cin >> userInput;
    
    switch(userInput)
    {
      case 'a': estimator.addLastRetrievedPosePair(); break;
      case 'e': estimator.calculateTransformation(); 
		
		cout<<endl<<"The calculated transformation matrix from hand coordinates to eye coordinates is:"<<endl;
		
		cout<<estimator.matrixH2E();
      
		break;
      case 'd': estimator.deleteLastAddedPosePair(); break;
      case 'c': estimator.clearAll(); break;
      case 'q': return 0;
      case 'l': {
		cout<<endl<<"Do you want to delete all previous data? (Y/n), Or type 'q' to go back to the main routine."<<endl;
		char deleteDataInput;
		bool deleteData = false;
		cin >> deleteDataInput;
		
		if( deleteDataInput=='Y' ) deleteData = true;
		else if( deleteDataInput=='q' ) break;
		
		cout<<endl<<"Please enter the name of the data file."<<endl;
		string loadFile;
		cin >> loadFile;
		if( estimator.loadFromFile( loadFile, deleteData ) ) cout<<endl<<"Data successfully loaded from file.";
		else cout<<endl<<"Loading data from file failed.";
		break;
		}
      case 's':	{
		cout<<endl<<"Please enter the name of the file you want to store the current data sets to."<<endl;
		string saveFile;
		cin >> saveFile;
		if( estimator.printToFile( saveFile ) ) cout<<endl<<"Data successfully saved to file."<<endl;
		else cout<<endl<<"Saving data to file failed."<<endl;
		break;
		}
      default: cout<<"Unknown entry. Try again."<<endl;
    };
    
  }
  
  
  return 0;
} 
