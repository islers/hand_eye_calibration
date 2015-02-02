/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*

math is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
math is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with math. If not, see <http://www.gnu.org/licenses/>.
*/ 

/// set of convenience functions and classes related to math

#pragma once
#include <utility>
#include <cmath>

namespace st_is
{
  
/** simple solver for the quadratic equation a*x² + bx + c = 0
*  Returns false if the roots are imaginary, otherwhise the two roots are stored in _roots - twice
*  the same value if only one root exists.
*/
bool roots( double _aCoeff, double _bCoeff, double _cCoeff, std::pair<double,double>& _roots );

}