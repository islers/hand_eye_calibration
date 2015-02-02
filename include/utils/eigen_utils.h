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

/// set of convenience functions and classes when working with Eigen
#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace st_is
{
  
/// returns the 3x3 matrix A whose multiplication A*b with a vector b is equivalent to the cross product _vec x b
Eigen::Matrix3d crossProdMatrix( Eigen::Vector3d _vec );

/// incomplete Dual Quaternion type
class DualQuaternion
{
public:
  DualQuaternion();
  /// initialize dual quaternion to represent given rotation and translation
  DualQuaternion( Eigen::Quaterniond _rot, Eigen::Vector3d _trans );
  
  Eigen::Quaterniond q;
  Eigen::Quaterniond q_prime;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
    
} 
