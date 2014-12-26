#include "hand_eye_calibration/DualQuaternionTransformationEstimator.hpp" 

DualQuaternionTransformationEstimator::DualQuaternionTransformationEstimator( ros::NodeHandle* _n )
{
  rosNode_ = _n;
  
  handSubscriber_ = rosNode_->subscribe("/hec/hand_position",1,&DualQuaternionTransformationEstimator::handListening,this);
  eyeSubscriber_ = rosNode_->subscribe("/hec/eye_position",1,&DualQuaternionTransformationEstimator::eyeListening,this);
  
  handRecorded_ = false;
  eyeRecorded_ = false;
  transformationCalculated_ = false;
}


DualQuaternionTransformationEstimator::~DualQuaternionTransformationEstimator()
{
  
}


void DualQuaternionTransformationEstimator::addLastRetrievedPosePair()
{  
  ROS_INFO("Saving last published poses for hand and eye...");
  
  int trialCount = 0;
  do
  {
    ros::spinOnce();
    
    if( !handRecorded_ )
    {
      ROS_INFO("No hand pose retrieved yet. Waiting...");
    }
    if( !eyeRecorded_ )
    {
      ROS_INFO("No eye pose retrieved yet. Waiting...");
    }
    if( trialCount++>=3 ) break;
    else if( !handRecorded_ || !eyeRecorded_ ) ROS_INFO("%i attempts remaining",3-trialCount);
    
  }while( rosNode_->ok() && ( !handRecorded_ || !eyeRecorded_ ) );
  
  if( handRecorded_ && eyeRecorded_ )
  {
    ros::Time currentTime = ros::Time::now();
    
    if( (currentTime - recordedHandTimeStamp_) >= ros::Duration(5.0) )
    {
      ROS_WARN("Adding hand pose that has been recorded %f seconds ago.", (currentTime-recordedHandTimeStamp_).toSec() );
    }
    if( (currentTime - recordedEyeTimeStamp_) >= ros::Duration(5.0) )
    {
      ROS_WARN("Adding eye pose that has been recorded %f seconds ago.", (currentTime-recordedHandTimeStamp_).toSec() );
    }
    
    posePairs_.push_back( pair<geometry_msgs::Pose, geometry_msgs::Pose>(bufferedHand_,bufferedEye_) );
    
    if( posePairs_.size()>=2 ) //estimation is possible
    {
      calculateTransformation(true);
      rotationEstimates_EH_.push_back( rot_EH_ );
      translationEstimates_E_t_EH_.push_back( E_trans_EH_ );
    }
  }
  
  
  return;
}



void DualQuaternionTransformationEstimator::deleteLastAddedPosePair()
{
  posePairs_.pop_back();
  if( rotationEstimates_EH_.size()>=1 ) rotationEstimates_EH_.pop_back();
  if( translationEstimates_E_t_EH_.size()>=1 ) translationEstimates_E_t_EH_.pop_back();
  return;
}


void DualQuaternionTransformationEstimator::calculateTransformation( bool _suppressWarnings )
{
  ROS_INFO("Calculating hand-eye transformation...");
  
  if( posePairs_.size()<=1 )
  {
    ROS_ERROR("At least two hand-eye pose pairs are necessary for computation.");
    ROS_INFO("Number of pose pairs currently available for computation: %i", posePairs_.size() );
    ROS_ERROR("Aborting transformation computation.");
    return;
  }
  else if( posePairs_.size()<20 )
  {
    ROS_WARN("For good results more than 20 hand-eye pose pairs are recommended. The computed transformation might be inaccurate.");
    ROS_INFO("Number of pose pairs currently available for computation: %i", posePairs_.size() );
    ROS_INFO("Going on with calculation...");
  }
  
  
  /* calculate movement transformations from pose to pose from base<->pose transformations and calculate
   * dual quaternions
   * input:	- hand: transformation base->hand
   * 		- eye:	transformation grid->camera
   * 
   * 
   *  dual quaternions are then hQ[i].first + epsilon*hQ[i].second
  */
  vector< pair<Eigen::Quaterniond,Eigen::Quaterniond>, Eigen::aligned_allocator<pair<Quaterniond, Quaterniond> > > hQ, cQ;
  for( int i=0; i<posePairs_.size()-1; i++ )
  {
    // transformation hand pose 1 (H1) -_> hand pose 2 (H2)
    
    Quaterniond rot_BH1 = geometryToEigen( posePairs_[i].first.orientation );
    Vector3d B_trans_BH1 = geometryToEigen( posePairs_[i].first.position );
    Quaterniond rot_H2B = geometryToEigen( posePairs_[i+1].first.orientation ).inverse();
    Vector3d B_trans_BH2 = geometryToEigen( posePairs_[i+1].first.position );
    
    Quaterniond rot_H2H1 = rot_H2B * rot_BH1;
    Vector3d H2_trans_H2H1 = rot_H2B * ( B_trans_BH1-B_trans_BH2 );
    pair<Quaterniond,Quaterniond> dualQuat_H2H1 = dualQuaternion( rot_H2H1, H2_trans_H2H1 );
    
    hQ.push_back( dualQuat_H2H1 );
    
    //transformation cam pose 1 (E1) -> cam pose 2 (E2)
    
    Quaterniond rot_GC1 = geometryToEigen( posePairs_[i].second.orientation ).inverse();
    Vector3d C1_trans_C1G = geometryToEigen( posePairs_[i].second.position );
    Quaterniond rot_C2G = geometryToEigen( posePairs_[i+1].second.orientation );
    Vector3d C2_trans_C2G = geometryToEigen( posePairs_[i+1].second.position );
    
    Quaterniond rot_C2C1 = rot_C2G * rot_GC1;
    Vector3d C2_trans_C2C1 = C2_trans_C2G - rot_C2C1*C1_trans_C1G;
    pair<Quaterniond,Quaterniond> dualQuat_C2C1 = dualQuaternion( rot_C2C1, C2_trans_C2C1 );
    
    cQ.push_back( dualQuat_C2C1 );
  }
  
  // build  S(i)...
  vector< Matrix<double,6,8>, Eigen::aligned_allocator<Eigen::Matrix<double,6,8> > > S;
  S.resize( hQ.size() );
  
  for( unsigned int i=0; i<hQ.size(); i++ )
  {
    Matrix<double,6,8> S_i;
    S[i] <<	hQ[i].first.vec()-cQ[i].first.vec()		,crossProdMatrix( hQ[i].first.vec()+cQ[i].first.vec() )		,Matrix<double,3,1>::Zero()			,Matrix<double,3,3>::Zero()
		,hQ[i].second.vec()-cQ[i].second.vec()		,crossProdMatrix( hQ[i].second.vec()+cQ[i].second.vec() )	,hQ[i].first.vec()-cQ[i].first.vec()		,crossProdMatrix( hQ[i].first.vec()+cQ[i].first.vec());
  }
  
  // build T
  Matrix<double,Dynamic,Dynamic> T; // 8*x-matrix
  for( unsigned int i=0; i<hQ.size(); i++ )
  {
    T.conservativeResize( 8, T.cols()+6 );
    T.block<8,6>(0,T.cols()-6) << S[i].transpose();
  }
  
  T.transposeInPlace();
  
  // null space of T: right null vectors
  JacobiSVD<MatrixXd,FullPivHouseholderQRPreconditioner> svd( T, ComputeFullV );
  MatrixXd V = svd.matrixV();
  
  Matrix<double,8,1> v_7 = V.col(6);
  Matrix<double,8,1> v_8 = V.col(7);
  
  
  /* decompose 8x1 v-vectors into two 4x1 vectors for
  /* 
  /* lambda_1*v_7 + lambda_2*v_8 = q; q: searched dual quaternion describing the hand-eye transformation
   * 
   * then
   * 
   * it follows from the constrains q^T*q = 1 and q^T*q' = 0:
   * 
   * lambda_1² * u_1^T * u_1 + 2*lambda_1*lambda_2*u_1^T*u_2 + lambda_2² * u_2^T * u_2 = 1
   * and
   * lambda² * u_1^T * v_1 + lambda_1*lambda_2*(u_1^T*v_2+u_2^T*v_1) + lambda_2² * u_2^T * v_2 = 0
   * 
   * with s = lambda_1 / lambda_2
   * 
   * lambda_1²/lambda_2² * u_1^T * v_1 + lambda_1*lambda_2/lambda_2² * (u_1^T * v_2 + u_2^T * v_1 ) + lambda_2² / lambda_2² * u_2^T * v_1 = 0
   * s² * u_1^T * v_1 + s * (u_1^T * v_2 + u_2^T * v_1 ) + u_2^T*v_2 = 0
   */
  
  Matrix<double,4,1> u_1 = v_7.topRows(4);
  Matrix<double,4,1> v_1 = v_7.bottomRows(4);
  
  Matrix<double,4,1> u_2 = v_8.topRows(4);
  Matrix<double,4,1> v_2 = v_8.bottomRows(4);
  
  // solve the quadratic equation for s
  pair<double,double> s;
  double a = (u_1.transpose() * v_1)[0];
  double b = (u_1.transpose() * v_2 + u_2.transpose() * v_1)[0];
  double c = (u_2.transpose() * v_2)[0];
  
  // equation has two real solutions of opposite sign (see paper)
  roots(a,b,c,s);
  
  // using 2nd equation:
  double leftSideCandidate_1 = (s.first*s.first*u_1.transpose()*u_1 + 2*s.first*u_1.transpose()*u_2 + u_2.transpose()*u_2)[0];
  double leftSideCandidate_2 = (s.second*s.second*u_1.transpose()*u_1 + 2*s.second*u_1.transpose()*u_2 + u_2.transpose()*u_2)[0];
  
  // use larger value
  double sSol, leftSide;
  if( leftSideCandidate_1>leftSideCandidate_2 )
  {
    sSol = s.first;
    leftSide = leftSideCandidate_1;
  }
  else
  {
    sSol = s.second;
    leftSide = leftSideCandidate_2;
  }
  
  // calculate lambdas
  double lambda_2 = sqrt(1/leftSide);
  double lambda_1 = sSol * lambda_2;
  
  // finally calculate resulting dual quaternion (w, x, y, z)^T
  Matrix<double,8,1> qVec = lambda_1 * v_7 + lambda_2 * v_8;
  
  pair<Quaterniond,Quaterniond> q;
  q.first = Quaterniond( qVec(0),qVec(1),qVec(2),qVec(3) ); // quaternion representing the rotation R_CH (hand to eye)
  q.second = Quaterniond( qVec(4), qVec(5), qVec(6), qVec(7) );
  
  // calculate the translation
  Quaterniond t_p = q.second*q.first.conjugate();
  Vector3d t = 2*t_p.vec(); // translation vector representing the translation from hand- to eye-frame in hand coordinates
  
  // save transformation
  rot_EH_ = q.first;
  E_trans_EH_ = rot_EH_*t;
  E_trans_EH_ = -E_trans_EH_;
  
  transformationCalculated_ = true;
  return;
}


geometry_msgs::Pose DualQuaternionTransformationEstimator::getHandToEye()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::getTransformation() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  
  geometry_msgs::Pose estimatedTransformation;
  
  estimatedTransformation.position.x = E_trans_EH_[0];
  estimatedTransformation.position.y = E_trans_EH_[1];
  estimatedTransformation.position.z = E_trans_EH_[2];
  estimatedTransformation.orientation.x = rot_EH_.x();
  estimatedTransformation.orientation.y = rot_EH_.y();
  estimatedTransformation.orientation.z = rot_EH_.z();
  estimatedTransformation.orientation.w = rot_EH_.w();
  
  return estimatedTransformation;
}


Matrix3d DualQuaternionTransformationEstimator::rotH2E()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::rotH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return rot_EH_.toRotationMatrix();
}


Matrix3d DualQuaternionTransformationEstimator::rotE2H()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::rotE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return rot_EH_.inverse().toRotationMatrix();
}


Vector3d DualQuaternionTransformationEstimator::transH2E()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::transH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return E_trans_EH_;
}


Vector3d DualQuaternionTransformationEstimator::transE2H()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::transE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  return -(rotH2E()*E_trans_EH_);
}


Matrix<double,4,4> DualQuaternionTransformationEstimator::matrixH2E()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::matrixH2E() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  Matrix<double,4,4> mH2E;
  
  Vector3d E_t_EH = E_trans_EH_;
  
  mH2E<<rotH2E(), E_t_EH, 0, 0, 0, 1;
  
  return mH2E;
}


Matrix<double,4,4> DualQuaternionTransformationEstimator::matrixE2H()
{
  if( !transformationCalculated_ ) ROS_WARN("DualQuaternionTransformationEstimator::matrixE2H() called but no transformation has been computed yet. The retrieved transformation has no significance.");
  Matrix<double,4,4> mE2H;
  
  Vector3d H_t_HE = - rotE2H()*E_trans_EH_;
  
  mE2H<<rotE2H(), H_t_HE, 0, 0, 0, 1;
  
  return mE2H;
}


void DualQuaternionTransformationEstimator::clearAll()
{
  ROS_INFO("Deleting all buffered hand-eye pose pairs.");
  posePairs_.clear();
  ROS_INFO("Number of pose pairs now available for computation: %i", posePairs_.size() );
  
  return;
}


void DualQuaternionTransformationEstimator::handListening( const geometry_msgs::PoseConstPtr& _newPose )
{  
  bufferedHand_ = *_newPose;
  handRecorded_ = true;
  recordedHandTimeStamp_ = ros::Time::now();
  return;
}


void DualQuaternionTransformationEstimator::eyeListening( const geometry_msgs::PoseConstPtr& _newPose )
{  
  bufferedEye_ = *_newPose;
  eyeRecorded_ = true;
  recordedEyeTimeStamp_ = ros::Time::now();
  return;
}


int DualQuaternionTransformationEstimator::count()
{
  return posePairs_.size();
}



bool DualQuaternionTransformationEstimator::roots( double _aCoeff, double _bCoeff, double _cCoeff, pair<double,double>& _roots )
{
  double discriminant = sqrt( _bCoeff*_bCoeff - 4*_aCoeff*_cCoeff );
  
  if( discriminant<0 ) return false;
  
  _roots.first = (-_bCoeff + discriminant ) / (2*_aCoeff);
  _roots.second = (-_bCoeff - discriminant ) / (2*_aCoeff);
  return true;
}



bool DualQuaternionTransformationEstimator::printToFile( string fileName_ )
{
  // create Mat arrays with the poses of hand and eye_position
  
  try
  {
    cv::Mat handPoses(7,posePairs_.size() ,CV_64FC1);
    cv::Mat eyePoses(7,posePairs_.size() ,CV_64FC1);
    
    for( int i=0; i<posePairs_.size(); i++ )
    {
      handPoses.at<double>(0,i) = posePairs_[i].first.orientation.x;
      handPoses.at<double>(1,i) = posePairs_[i].first.orientation.y;
      handPoses.at<double>(2,i) = posePairs_[i].first.orientation.z;
      handPoses.at<double>(3,i) = posePairs_[i].first.orientation.w;
      handPoses.at<double>(4,i) = posePairs_[i].first.position.x;
      handPoses.at<double>(5,i) = posePairs_[i].first.position.y;
      handPoses.at<double>(6,i) = posePairs_[i].first.position.z;
      
      eyePoses.at<double>(0,i) = posePairs_[i].second.orientation.x;
      eyePoses.at<double>(1,i) = posePairs_[i].second.orientation.y;
      eyePoses.at<double>(2,i) = posePairs_[i].second.orientation.z;
      eyePoses.at<double>(3,i) = posePairs_[i].second.orientation.w;
      eyePoses.at<double>(4,i) = posePairs_[i].second.position.x;
      eyePoses.at<double>(5,i) = posePairs_[i].second.position.y;
      eyePoses.at<double>(6,i) = posePairs_[i].second.position.z;
    }
    
    cv::Mat estimatedTransformations( 7, rotationEstimates_EH_.size(), CV_64FC1 );
    for( int i=0; i<rotationEstimates_EH_.size(); i++ )
    {
      estimatedTransformations.at<double>(0,i) = rotationEstimates_EH_[i].x();
      estimatedTransformations.at<double>(1,i) = rotationEstimates_EH_[i].y();
      estimatedTransformations.at<double>(2,i) = rotationEstimates_EH_[i].z();
      estimatedTransformations.at<double>(3,i) = rotationEstimates_EH_[i].w();
      estimatedTransformations.at<double>(4,i) = translationEstimates_E_t_EH_[i].x();
      estimatedTransformations.at<double>(5,i) = translationEstimates_E_t_EH_[i].y();
      estimatedTransformations.at<double>(6,i) = translationEstimates_E_t_EH_[i].z();
    }
    
    cv::FileStorage outputFile( fileName_, cv::FileStorage::WRITE );
    
    outputFile<<"handPoses"<<handPoses;
    outputFile<<"eyePoses"<<eyePoses;
    outputFile<<"estimatedTransformations"<<estimatedTransformations;
  }
  catch(...)
  {
    ROS_ERROR("DualQuaternionTransformationEstimator::printToFile::failed.");
    return 0;
  }
  
  return 1;
}



bool DualQuaternionTransformationEstimator::loadFromFile( string fileName_, bool destroyOldData_ )
{
  try
  {
    cv::FileStorage inputFile( fileName_, cv::FileStorage::READ );
    
    cv::Mat handPoses( cv::Size(), CV_64FC1 );
    cv::Mat eyePoses( cv::Size(), CV_64FC1 );
    cv::Mat estimatedTransformations( cv::Size(), CV_64FC1 );
    
    inputFile["handPoses"] >> handPoses;
    inputFile["eyePoses"] >> eyePoses;
    inputFile["estimatedTransformations"] >> estimatedTransformations;
    
    if( handPoses.cols!=eyePoses.cols || handPoses.rows!=7 || eyePoses.rows!=7 || handPoses.cols==0 || estimatedTransformations.rows!=7 || estimatedTransformations.cols!=(handPoses.cols-1) )
    {
       ROS_ERROR("DualQuaternionTransformationEstimator::loadFromFile::failed::The input file %s did not contain valid cv::Mat matrices.",fileName_.c_str() );
       return 0;
    }
    
    if( destroyOldData_ )
    {
      posePairs_.clear();
      rotationEstimates_EH_.clear();
      translationEstimates_E_t_EH_.clear();
    }
    
    for( int i=0; i<handPoses.cols; i++ )
    {
     geometry_msgs::Pose handPoseToAdd, eyePoseToAdd;
     
     handPoseToAdd.orientation.x = handPoses.at<double>(0,i);
     handPoseToAdd.orientation.y = handPoses.at<double>(1,i);
     handPoseToAdd.orientation.z = handPoses.at<double>(2,i);
     handPoseToAdd.orientation.w = handPoses.at<double>(3,i);
     handPoseToAdd.position.x = handPoses.at<double>(4,i);
     handPoseToAdd.position.y = handPoses.at<double>(5,i);
     handPoseToAdd.position.z = handPoses.at<double>(6,i);
     
     eyePoseToAdd.orientation.x = eyePoses.at<double>(0,i);
     eyePoseToAdd.orientation.y = eyePoses.at<double>(1,i);
     eyePoseToAdd.orientation.z = eyePoses.at<double>(2,i);
     eyePoseToAdd.orientation.w = eyePoses.at<double>(3,i);
     eyePoseToAdd.position.x = eyePoses.at<double>(4,i);
     eyePoseToAdd.position.y = eyePoses.at<double>(5,i);
     eyePoseToAdd.position.z = eyePoses.at<double>(6,i);
      
      posePairs_.push_back( pair<geometry_msgs::Pose, geometry_msgs::Pose>( handPoseToAdd,eyePoseToAdd ) );
    }
    for( int i=0; i<estimatedTransformations.cols; i++ )
    {
      Quaterniond newRotationEstimate;
      newRotationEstimate.x() = estimatedTransformations.at<double>(0,i);
      newRotationEstimate.y() = estimatedTransformations.at<double>(1,i);
      newRotationEstimate.z() = estimatedTransformations.at<double>(2,i);
      newRotationEstimate.w() = estimatedTransformations.at<double>(3,i);
      Vector3d newTranslationEstimate;
      newTranslationEstimate.x() = estimatedTransformations.at<double>(4,i);
      newTranslationEstimate.y() = estimatedTransformations.at<double>(5,i);
      newTranslationEstimate.z() = estimatedTransformations.at<double>(6,i);
      
      rotationEstimates_EH_.push_back(newRotationEstimate);
      translationEstimates_E_t_EH_.push_back(newTranslationEstimate);
      
    }
    
  }
  catch(...)
  {
    ROS_ERROR("DualQuaternionTransformationEstimator::loadFromFile::failed.");
    return 0;
  }
  return 1;
}



Matrix3d DualQuaternionTransformationEstimator::crossProdMatrix( Vector3d _vec )
{
  Matrix3d ret;
  ret<< 0,		-_vec.z(),	_vec.y(),
	_vec.z(),	0,		-_vec.x(),
	-_vec.y(),	_vec.x(),	0;
  return ret;
}


Eigen::Vector3d DualQuaternionTransformationEstimator::geometryToEigen( const geometry_msgs::Point& _vec )
{
  Eigen::Vector3d output;
  output.x() = _vec.x;
  output.y() = _vec.y;
  output.z() = _vec.z;
  return output;
}


Eigen::Quaterniond DualQuaternionTransformationEstimator::geometryToEigen( const geometry_msgs::Quaternion& _quat )
{
  Eigen::Quaterniond output;
  output.x() = _quat.x;
  output.y() = _quat.y;
  output.z() = _quat.z;
  output.w() = _quat.w;
  return output;
}


pair<Eigen::Quaterniond,Eigen::Quaterniond> DualQuaternionTransformationEstimator::dualQuaternion( Eigen::Quaterniond _rot, Eigen::Vector3d _trans )
{
  Eigen::Quaterniond q, qPrime;
  
  q = _rot.normalized(); // just to ensure normalization
  
  Vector3d qAxis = q.vec();
  
  Vector3d qPrimeAxis = 0.5*( q.w()*_trans + _trans.cross(qAxis) );
  double qPrimeW = -0.5*qAxis.dot(_trans);
  
  qPrime.x() = qPrimeAxis.x();
  qPrime.y() = qPrimeAxis.y();
  qPrime.z() = qPrimeAxis.z();
  qPrime.w() = qPrimeW;
  
  return pair<Quaterniond,Quaterniond>(q,qPrime);
}