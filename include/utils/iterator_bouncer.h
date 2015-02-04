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

/// wrapper class for random access iterators that allows infinite incrementation or decrementation by letting the iterator bounce between two ends.
/** The class internally holds another iterator it (random access) and increments/decrements it just
 * as iterators do. However, if the iterator reaches one of the two ends specified it bounces: This means
 * that the incrementation direction is inversed and it++ actually means decrementation internally, it-- means
 * incrementation. Each time the iterator bounces the .bounced() - flag is set. It is unset after the next incrementation (or decrementation if decrementation caused the bounce). There are two bounce types: By default
 * the last iterator position in the range is passed only once, that is on an array {2,3,4,5}, the iterator
 * would pass the positions in the following order: 2,3,4,5,4,3,2,3,4,5,... If the pass_border_twice - flag is set, the order becomes: 2,3,4,5,5,4,3,2,2,3,4,5,5,4,...
 * IteratorBouncer wraps the iterator it holds, which means it behaves exactly as the type it holds, apart from
 * the bouncing behaviour.
 * If the iterator is initialized to a position not inside the specified range, a std::range_error is thrown.
 */

#include <iterator>
#include <stdexcept>
#include <string>

#define TEMPRAIt template<class RAIt>



namespace st_is
{
  /* *******************************
   * Header
   * ******************************/
  
  TEMPRAIt
  class IteratorBouncer: public std::iterator<std::bidirectional_iterator_tag,RAIt>
  {
  public:
    IteratorBouncer()
    {
    }
    /// constructor
    /** completely initializes the iterator
     * @param _init_pos initial position the internal iterator points to
     * @param _lower_bound lower bound iterator (lowest iterator that is part of the iterated set)
     * @param _upper_bound upper bound iterator (highest iterator that is part of the iterated set)
     * @throws std::range_error if the lower bound is higher than the upper bound or the initialized position doesn't lie in between the bounds (it may be equal to one of the bounds!)
     */
    IteratorBouncer( RAIt _init_pos, RAIt _lower_bound, RAIt _upper_bound );
    
    ///constructor
    /** initializes the iterator with bounds but without initial position
     * @param _lower_bound lower bound iterator (lowest iterator that is part of the iterated set)
     * @param _upper_bound upper bound iterator (highest iterator that is part of the iterated set)
     * @throws std::range_error if the lower bound is higher than the upper bound
     */
    IteratorBouncer( RAIt _lower_bound, RAIt _upper_bound );
    
    /// set the position of the internal iterator
    /**
     * @param _new_pos new iterator the IteratorBouncer wraps
     * @throws std::range_error if the new position doesn't lie inside the bounds
     */
    void setPos( RAIt _new_pos );
    
    /// set new bounds
    /**
     * @param _lower_bound lower bound iterator (lowest iterator that is part of the iterated set)
     * @param _upper_bound upper bound iterator (highest iterator that is part of the iterated set)
     * @throws std::range_error if the lower bound is higher than the upper bound
     */
    void setBounds( RAIt _lower_bound, RAIt _upper_bound );
        
    typename std::iterator_traits<RAIt>::value_type& operator*();
    typename std::iterator_traits<RAIt>::pointer operator->();
    
    IteratorBouncer<RAIt>& operator++();
    IteratorBouncer<RAIt> operator++(int);
    IteratorBouncer<RAIt>& operator--();
    IteratorBouncer<RAIt> operator--(int);
    
    /// return the upper limit
    RAIt upperLimit();
    
    /// return the lower limit
    RAIt lowerLimit();
    
    /// whether the IteratorBouncer bounced during the last incrementation or decrementation or not
    bool bounced();
    
    bool pass_border_twice; // whether the border element is passed twice, once before and once after bouncing or not, default is false
    
    /// splits the iterator at the current position - splitting is deactivated if the new border equals the border the iterator would have in the unsplit case (which is its lower_bound)
    void split();
    
    /// sets a resolving split at the current internal position that resolves after one iteration
    /** e.g. split at 4 in space (1..6): for incrementing: 4,5,6,3,2,1,2,3,4,5,6,5,4,3,2,1,2,3,4,5,6,...
     */
    void resolvingSplit();
    
    /// whether the iterator is split or not
    bool isSplit();
    
    /// returns the iterator border - empty iterator if iterator not split
    RAIt iteratorBorder();
    
  private:
    RAIt pos_; /// internal iterator that is being moved
    RAIt lower_bound_; /// lower bound iterator (lowest iterator that is part of the iterated set)
    RAIt upper_bound_; /// upper bound iterator (highest iterator that is part of the iterated set)
    
    bool bounced_; /// whether the IteratorBouncer bounced_ during the last incrementation or decrementation or not
    bool is_reversed_; /// after one bounce the iteration is reversed, after two bounces it is normal again
    bool split_space_; /// true if the iterator border is not at the lower and upper bound but somewhere in the array
    bool split_resolves_; /// whether splits resolve or not
    RAIt iterator_border_; /// border of the iterator if splitSpace_=true: iterator_border equals the new lower bound, iterator_border-1 the new upper bound
    
    void moveUp(); /// moves the iterator upwards (can bounce)
    void moveDown(); /// moves the iterator downwards (can bounce)
    
    /// returns true if the current iterator points to the begin (split space considered)
    bool atBegin();
    /// returns true if the current iterator points to the end (split space considered)
    bool atEnd();
    
    /// if the iterator exceeds the boundaries, it is reset to the boundary
    void enforceBoundaries();
  };
  
  
  /* ***************************
   * Functions
   * **************************/
  
  TEMPRAIt
  IteratorBouncer<RAIt>::IteratorBouncer( RAIt _init_pos, RAIt _lower_bound, RAIt _upper_bound ):
    pass_border_twice(false),
    bounced_(false),
    is_reversed_(false),
    split_space_(false),
    split_resolves_(false)
  {
    if( _lower_bound > _upper_bound )
    {
      std::range_error e("iterator_bouncer.h::line "+std::to_string(__LINE__)+"::IteratorBouncer<RAIt>::IteratorBouncer( RAIt _init_pos, RAIt _lower_bound, RAIt _upper_bound ):: Called with invalid range (lower bound higher than upper bound)");
      throw e;
    }
    else if( _init_pos > _upper_bound || _init_pos < _lower_bound )
    {
      std::range_error e("iterator_bouncer.h::line "+std::to_string(__LINE__)+"::IteratorBouncer<RAIt>::IteratorBouncer( RAIt _init_pos, RAIt _lower_bound, RAIt _upper_bound ):: Called with invalid position: _init_pos doesn't lie in the given range.");
      throw e;
    }
    pos_ = _init_pos;
    lower_bound_ = _lower_bound;
    upper_bound_ = _upper_bound;
  }
  
  TEMPRAIt
  IteratorBouncer<RAIt>::IteratorBouncer( RAIt _lower_bound, RAIt _upper_bound ):
    pass_border_twice(false),
    bounced_(false),
    is_reversed_(false),
    split_space_(false),
    split_resolves_(false)
  {
    if( _lower_bound > _upper_bound )
    {
      std::range_error e("iterator_bouncer.h::line "+std::to_string(__LINE__)+"::IteratorBouncer( RAIt _lower_bound, RAIt _upper_bound ):: Called with invalid range (lower bound higher than upper bound)");
      throw e;
    }
    lower_bound_ = _lower_bound;
    upper_bound_ = _upper_bound;
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::setPos( RAIt _new_pos )
  {
    if( _new_pos > upper_bound_ || _new_pos < lower_bound_ )
    {
      std::range_error e("iterator_bouncer.h::line "+std::to_string(__LINE__)+"::void IteratorBouncer<RAIt>::setPos( RAIt _new_pos ):: Called with invalid position: _new_pos doesn't lie in the range specified for the IteratorBouncer.");
      throw e;
    }
    pos_ = _new_pos;
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::setBounds( RAIt _lower_bound, RAIt _upper_bound )
  {
    if( _lower_bound > _upper_bound )
    {
      std::range_error e("iterator_bouncer.h::line "+std::to_string(__LINE__)+"::void IteratorBouncer<RAIt>::setBounds( RAIt _lower_bound, RAIt _upper_bound ):: Called with invalid range (lower bound higher than upper bound)");
      throw e;
    }
    lower_bound_ = _lower_bound;
    upper_bound_ = _upper_bound;
  }
  
  TEMPRAIt
  typename std::iterator_traits<RAIt>::value_type& IteratorBouncer<RAIt>::operator*()
  {
    return (*pos_);
  }
  
  TEMPRAIt
  typename std::iterator_traits<RAIt>::pointer IteratorBouncer<RAIt>::operator->()
  {
    return pos_.operator->();
  }
  
  TEMPRAIt
  IteratorBouncer<RAIt>& IteratorBouncer<RAIt>::operator++()
  {
    if( !is_reversed_ ) moveUp();
    else moveDown();
    
    return (*this);
  }
  
  TEMPRAIt
  IteratorBouncer<RAIt> IteratorBouncer<RAIt>::operator++(int)
  {
    IteratorBouncer<RAIt> copy(*this);
    operator++();
    return copy;
  }
  
  TEMPRAIt
  IteratorBouncer<RAIt>& IteratorBouncer<RAIt>::operator--()
  {
    if( !is_reversed_ ) moveDown();
    else moveUp();
    
    return (*this);
  }
  
  TEMPRAIt
  IteratorBouncer<RAIt> IteratorBouncer<RAIt>::operator--(int)
  {
    IteratorBouncer<RAIt> copy(*this);
    operator--();
    return copy;
  }
  
  TEMPRAIt
  RAIt IteratorBouncer<RAIt>::upperLimit()
  {
    return upper_bound_;
  }
  
  TEMPRAIt
  RAIt IteratorBouncer<RAIt>::lowerLimit()
  {
    return lower_bound_;
  }
  
  TEMPRAIt
  bool IteratorBouncer<RAIt>::bounced()
  {
    return bounced_;
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::split()
  {
    split_resolves_ = false;
    
    if( pos_==lower_bound_ )
    {
      split_space_ = false;
    }
    else
    {
      iterator_border_ = pos_;
      split_space_ = true;
    }
    return;
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::resolvingSplit()
  {
    if( pos_<=lower_bound_ )
    {
      pos_=lower_bound_;
      split_space_ = false;
    }
    else
    {
      iterator_border_ = pos_;
      split_space_ = true;
      split_resolves_ = true;
    }
    return;
  }
  
  TEMPRAIt
  bool IteratorBouncer<RAIt>::isSplit()
  {
    return split_space_;
  }
  
  TEMPRAIt
  RAIt IteratorBouncer<RAIt>::iteratorBorder()
  {
    if( split_space_ ) return pos_;
    else return RAIt();
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::moveUp()
  {
    if( atEnd() )
    {
      if( !pass_border_twice  )
      {
	if( !split_space_ )
	{
	  if( pos_ != lower_bound_ ) pos_--;
	}
	else
	{
	  if( pos_ != lower_bound_ )
	    pos_--;
	  else
	  {
	    if( !split_resolves_ )
	      pos_ = upper_bound_;
	    else
	    {
	      pos_ = iterator_border_-1;
	      split_space_ = false;
	    }
	  }
	}
      }
      
      bounced_ = true;
      is_reversed_ = !is_reversed_;
    }
    else
    {
      bounced_ = false;
      if( !split_space_ )
      {
	pos_++;
      }
      else
      {
	if( pos_!=upper_bound_ )
	  pos_++;
	else
	{
	  if( !split_resolves_ )
	    pos_ = lower_bound_;
	  else
	  {
	    pos_ = iterator_border_-1;
	    split_space_ = false;
	    is_reversed_ = !is_reversed_;
	  }
	}
      }
    }
    enforceBoundaries();
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::moveDown()
  {
    if( atBegin() )
    {
      if( !pass_border_twice )
      {
	if( !split_space_ )
	{
	  if( pos_ != upper_bound_ ) pos_++;
	}
	else
	{
	  if( pos_ != upper_bound_ )
	    pos_++;
	  else
	  {
	    if( !split_resolves_ )
	      pos_ = lower_bound_;
	    else
	    {
	      pos_ = iterator_border_;
	      split_space_ = false;
	    }
	  }
	}
      }
      
      bounced_ = true;
      is_reversed_ = !is_reversed_;
    }
    else
    {
      bounced_ = false;
      
      if( !split_space_ )
	pos_--;
      else
      {
	if( pos_ != lower_bound_ )
	  pos_--;
	else
	{
	  if( !split_resolves_ )
	    pos_ = upper_bound_;
	  else
	  {
	    pos_ = iterator_border_;
	    split_space_ = false;
	    is_reversed_ = !is_reversed_;
	  }
	}
      }
    }
    enforceBoundaries();
  }
  
  TEMPRAIt
  bool IteratorBouncer<RAIt>::atBegin()
  {
    if( !split_space_ )
    {
      return ( pos_ == lower_bound_ );
    }
    else
    {
      return ( pos_ == iterator_border_ );
    }
  }
  
  TEMPRAIt
  bool IteratorBouncer<RAIt>::atEnd()
  {
    if( !split_space_ )
    {
      return ( pos_ == upper_bound_ );
    }
    else
    {
      return ( pos_ == (iterator_border_-1) );
    }
  }
  
  TEMPRAIt
  void IteratorBouncer<RAIt>::enforceBoundaries()
  {
    if( pos_>upper_bound_ )
      pos_=upper_bound_;
    else if( pos_<lower_bound_ )
      pos_=lower_bound_;
  }
}

#undef TEMPRAIt