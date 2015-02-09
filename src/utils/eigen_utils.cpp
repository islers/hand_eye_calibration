 /* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*

eigen_utils is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
eigen_utils is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with eigen_utils. If not, see <http://www.gnu.org/licenses/>.
*/ 
 
#include "utils/eigen_utils.h"

using namespace Eigen;

namespace st_is
{
  
Matrix3d crossProdMatrix( Vector3d _vec )
{
  Matrix3d ret;
  ret<< 0,		-_vec.z(),	_vec.y(),
	_vec.z(),	0,		-_vec.x(),
	-_vec.y(),	_vec.x(),	0;
  return ret;
}

Eigen::Vector3d applyTransform( Eigen::Vector3d& _vec, Eigen::Quaterniond& _rotation, Eigen::Vector3d& _translation )
{
  return _rotation*_vec + _translation;
}

CoordinateTransformation::CoordinateTransformation()
{
  
}

CoordinateTransformation::CoordinateTransformation( const Eigen::Quaterniond& _rotation, const Eigen::Vector3d& _translation )
{
  rotation = _rotation;
  translation = _translation;
}

CoordinateTransformation CoordinateTransformation::inv()
{
  Eigen::Quaterniond inv_rot = rotation.conjugate();
  Eigen::Vector3d inv_translation = inv_rot*(-translation);
  return CoordinateTransformation( inv_rot, inv_translation );
}

CoordinateTransformation CoordinateTransformation::operator*( const CoordinateTransformation _toMultiply )
{
  Eigen::Quaterniond new_rot = rotation*_toMultiply.rotation;
  Eigen::Vector3d new_trans = rotation*_toMultiply.translation + translation;
  return CoordinateTransformation( new_rot, new_trans );
}

Eigen::Vector3d CoordinateTransformation::operator*( const Eigen::Vector3d _toMultiply )
{
  return rotation*_toMultiply + translation;
}


DualQuaternion::DualQuaternion()
{
  
}

DualQuaternion::DualQuaternion( Eigen::Quaterniond _rot, Eigen::Vector3d _trans )
{
  q = _rot.normalized(); // just to ensure normalization
  
  // by the screw congruence theorem q and q' one must be equal for hand eye calibration for both the eye and the hand movement. since the rotation represented by quaternion q is equal to -q, enforcing q_1>=0
  if( q.w()<0 )
  {
    q.w() = - q.w();
    q.x() = -q.x();
    q.y() = -q.y();
    q.z() = -q.z();
  }
  
  Vector3d qAxis = q.vec();
  
  Vector3d qPrimeAxis = 0.5*( q.w()*_trans + _trans.cross(qAxis) );
  double qPrimeW = -0.5*qAxis.dot(_trans);
  
  q_prime.x() = qPrimeAxis.x();
  q_prime.y() = qPrimeAxis.y();
  q_prime.z() = qPrimeAxis.z();
  q_prime.w() = qPrimeW;
  
  q_prime = q_prime;
}
    
} 
