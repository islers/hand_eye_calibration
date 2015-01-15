 /* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
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

#pragma once

/// class to iterate through a multidimensional space spanned by multiple IteratorBouncers with the pass_border_twice flag set.
/** Dimensions can be dynamically added. Incrementing the MultiDimensionalSpaceIterator leads to it
 * counting through the currently constructed space. A dimension added later is of one order higher
 * than the one added directly before. That is, if the space spanned would correspond to the space
 * of integral numbers the first dimension would correspond to the ones, the second to the 
 * tens etc. Since the space is spanned by IteratorBouncers with the pass_border_twice it is counted through such that the position in space changes at each iteration step only in one dimension. If the whole space
 * was covered, ie all internally used IteratorBouncers have their bounced() flag set, then a reachedTop()
 * flag is being set. An example iteration on two dimensions in integral space would be:
 * 00, 01, 02, 03, 04, 05, 06, 07, 08, 09, 19, 18, 17, .., 12, 11, 10, 20, 21, 22, etc...
 */
#include "utils/iterator_bouncer.h"
#include <vector>
#include <string>

#define TEMPRAIt template<class RAIt>

namespace st_is
{
  /* *******************************
   * Header
   * *******************************/
  
  TEMPRAIt
  class MultiDimensionalSpaceIterator: public std::iterator<std::bidirectional_iterator_tag,RAIt>
  {
  public:
    MultiDimensionalSpaceIterator():
      use_named_dim_(true)
    {
      
    }
    
    /// add new dimension
    /** Add a new dimension. Optionally you can give it a name
     * @param _new_dim the new dimension
     * @param _name the name for the new dimension
     * @throws std::runtime_error If the name given for the dimension already exists ("" will disable dimension naming for all dimensions)
     */
    void addDim( IteratorBouncer<RAIt>* _new_dim, std::string _name="" );
    
    /// returns the number of dimensions the space currently has
    unsigned int getDimensionality();
    
    /// returns the value of the iterator pointed at by the iterator of dimension _dim_name
    /** @param _dim_name The name of the dimension
     * @return An empty value_type if no dimension with the given name was found
     * @throws std::runtime_error If dimension naming is disabled, that is use_named_dim_=false
     */
    typename std::iterator_traits<RAIt>::value_type getDimValue( std:: string _dim_name );
    
    /// returns the value of the iterator pointed at by the iterator with order _order
    /** @param _order Order of the dimension (the first has order 0
     * @throws std::range_error If the given order _order is too high
     */
    typename std::iterator_traits<RAIt>::value_type getDimValue( unsigned int _order );
    
    /// returns a reference to the value of the iterator pointed at by the iterator with order _order
    /** @param _order Order of the dimension (the first has order 0
     * @throws std::range_error If the given order _order is too high
     */
    typename std::iterator_traits<RAIt>::value_type& operator[]( unsigned int _order );
    
    /// returns the internal position and dimension array
    std::vector< IteratorBouncer<RAIt> >& operator*();
    
    /// returns a pointer to the internal position dimension array
    std::vector< IteratorBouncer<RAIt> >* operator->();
        
    MultiDimensionalSpaceIterator<RAIt>& operator++();
    MultiDimensionalSpaceIterator<RAIt> operator++(int);
    MultiDimensionalSpaceIterator<RAIt>& operator--();
    MultiDimensionalSpaceIterator<RAIt> operator--(int);
    
    /// returns true if the highest point in the space is reached, defined as the position where the bounced() flag is set for all internal IteratorBouncer.
    /** Note that this position may change if the range of the internal IteratorBouncers changes */
    bool reachedTop();
    
  private:
    std::vector< IteratorBouncer<RAIt> > position_; /// vector pointing to the current position in iterator space
    std::vector< std::string > dim_names_; /// names of the dimensions
    bool use_named_dim_; /// if any dimension was added without a name, this will be set to false
  };
  
  
  /* ********************************
   * Functions
   * ********************************/
  
  TEMPRAIt
  void MultiDimensionalSpaceIterator<RAIt>::addDim( IteratorBouncer<RAIt>* _new_dim, std::string _name )
  {
    if ( _name=="" ) use_named_dim_ = false;
    else if( use_named_dim_ )
    {
      for( std::size_t i = 0; i<dim_names_.size(); i++ )
      {
	if( dim_names_[i]==_name )
	{
	  std::runtime_error e("multi_dimensional_space_iterator.h::110::void MultiDimensionalSpaceIterator<RAIt>::addDim( IteratorBouncer<RAIt>* _new_dim, std::string _name="" )::The name '"+_name+"' is not unique.");
	  throw e;
	}
      }
      dim_names_.push_back(_name);
    }
    
    position_.push_back( *_new_dim );
    position_.back().pass_border_twice = true;
  }
  
  TEMPRAIt
  unsigned int MultiDimensionalSpaceIterator<RAIt>::getDimensionality()
  {
    return position_.size();
  }
  
  TEMPRAIt
  typename std::iterator_traits<RAIt>::value_type MultiDimensionalSpaceIterator<RAIt>::getDimValue( std:: string _dim_name )
  {
    if( !use_named_dim_ )
    {
      std::runtime_error e("multi_dimensional_space_iterator.h::133::typename std::iterator_traits<RAIt>::value_type MultiDimensionalSpaceIterator<RAIt>::getDimValue( std:: string _dim_name )::function called but dimension naming was deactivated, because of an added dimension without name.");
      throw e;
    }
    
    for( std::size_t i = 0; i<dim_names_.size(); i++ )
    {
      if( dim_names_[i]==_dim_name )
      {
	return (*position_[i]);
      }
    }
    return std::iterator_traits<RAIt>::value_type();
  }
  
  TEMPRAIt
  typename std::iterator_traits<RAIt>::value_type MultiDimensionalSpaceIterator<RAIt>::getDimValue( unsigned int _order )
  {
    if( _order >= position_.size() )
    {
      std::range_error e("multi_dimensional_space_iterator.h::152::typename std::iterator_traits<RAIt>::value_type MultiDimensionalSpaceIterator<RAIt>::getDimValue( unsigned int _order ):: The order by which the function was called exceeds the order of the highest dimension.");
      throw e;
    }
    
    return *(position_[_order]);
  }
  
  TEMPRAIt
  typename std::iterator_traits<RAIt>::value_type& MultiDimensionalSpaceIterator<RAIt>::operator[]( unsigned int _order )
  {
    if( _order >= position_.size() )
    {
      std::range_error e("multi_dimensional_space_iterator.h::152::typename std::iterator_traits<RAIt>::value_type MultiDimensionalSpaceIterator<RAIt>::getDimValue( unsigned int _order ):: The order by which the function was called exceeds the order of the highest dimension.");
      throw e;
    }
    
    return *(position_[_order]);
  }
  
  TEMPRAIt
  std::vector< IteratorBouncer<RAIt> >& MultiDimensionalSpaceIterator<RAIt>::operator*()
  {
    return position_;
  }
  
  TEMPRAIt
  std::vector< IteratorBouncer<RAIt> >* MultiDimensionalSpaceIterator<RAIt>::operator->()
  {
    return &position_;
  }
  
  TEMPRAIt
  MultiDimensionalSpaceIterator<RAIt>& MultiDimensionalSpaceIterator<RAIt>::operator++()
  {
    for( std::size_t dim = 0; dim < position_.size(); dim++ )
    {
      position_[dim]++;
      if( !position_[dim].bounced() ) break;
    }
    return (*this);
  }
  
  TEMPRAIt
  MultiDimensionalSpaceIterator<RAIt> MultiDimensionalSpaceIterator<RAIt>::operator++(int)
  {
    MultiDimensionalSpaceIterator<RAIt> copy(*this);
    operator++();
    return copy;
  }
  
  TEMPRAIt
  MultiDimensionalSpaceIterator<RAIt>& MultiDimensionalSpaceIterator<RAIt>::operator--()
  {
    for( std::size_t dim=0; dim < position_.size(); dim++ )
    {
      position_[dim]--;
      if( !position_[dim].bounced() ) break;
    }
  }
  
  TEMPRAIt
  MultiDimensionalSpaceIterator<RAIt> MultiDimensionalSpaceIterator<RAIt>::operator--(int)
  {
    MultiDimensionalSpaceIterator<RAIt> copy(*this);
    operator--();
    return copy;
  }    
  
  TEMPRAIt
  bool MultiDimensionalSpaceIterator<RAIt>::reachedTop()
  {
    bool reached_top = true;
    for( std::size_t dim=0; dim < position_.size(); dim++ ) reached_top = reached_top && position_[dim].bounced();
    return reached_top;
  }
}


#undef TEMPRAIt