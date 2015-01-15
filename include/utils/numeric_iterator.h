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

#include <iterator>

/// Class to iterate numeric types.
/** Standard random access iterator class to iterate over numeric types (int,double,etc) 
 * with fixed step sizes. So basically their numerical space is discretized in steps of 
 * predefined size over which an iteration can be made just as with any other iterator.
 * All numerical values lying inside the same cell reprented by an iterator are equal when
 * being compared. Iterators with unequal base or step size are never equal and always return
 * false on all comparisons but the inequality.
 * Instead of using a constant iterator which would make sense since the iterated sets are constant
 * and non-writable (e.g. the set of real numbers reprented by double), accessing and writing
 * to the element pointed at by the iterator changes the internal value and thus the iterator.
 */
#define TEMPT template<class T>

namespace st_is
{
  
  /**********************//**
   * header
   * **********************/
  
  TEMPT
  class NumericIterator: public std::iterator<std::random_access_iterator_tag,T>
  {
  public:
    /// constructor
    /** Initializes the numeric iterator.
     * @param _base start point of iterator, used as base and initial value, also sets the grid for space discretization
     * @param _step_size sets the increment
     */
    NumericIterator( T _base=T(), T _step_size=T() ):value_(_base),base_(_base),step_size_(_step_size)
    {
      
    }
    
    /// constructor
    /** Initializes the numeric iterator.
     * @param _base start point of iterator, used as base, also sets the grid for space discretization
     * @param _step_size sets the increment
     * @param _value sets the initial value
     */
    NumericIterator( T _base, T _step_size, T _value ):value_(_value),base_(_base),step_size_(_step_size)
    {
      
    }
    
    
    /// sets a new base
    /** sets a new base but does not change the current value the iterator points to
     * @param _new_base the new base
     */
    void setBase( T _new_base )
    {
      base_=_new_base;
    }
    
    /// sets a new step size
    void setStep( T _new_step )
    {
      step_size_=_new_step;      
    }
    
    
    explicit operator T() const { return value_; }
    
    bool operator==( const NumericIterator<T>& _to_compare ) const;
    bool operator!=( const NumericIterator<T>& _to_compare ) const;
    
    T& operator*();
    T* operator->();
    
    NumericIterator<T>& operator++();
    NumericIterator<T> operator++(int);
    NumericIterator<T>& operator--();
    NumericIterator<T> operator--(int);
    
    NumericIterator<T> operator+(int _offset);
    NumericIterator<T> operator-(int _offset);
    
    bool operator<( const NumericIterator<T>& _to_compare ) const;
    bool operator>( const NumericIterator<T>& _to_compare ) const;
    bool operator<=( const NumericIterator<T>& _to_compare ) const;
    bool operator>=( const NumericIterator<T>& _to_compare ) const;
    
    void operator+=( int _offset );
    void operator-=( int _offset );
    
    /// comment: This function does not really make sense for the numeric iterator type
    /** this function doesn't make sense at all since you cannot change */
  private: T& operator[]( int _offset );
    
    
  private:
    T value_; /// current value
    T base_; /// base of one field (lower end of one numeric range represented by the iterator
    T step_size_; /// iterator step size
    
    /// norms the value, that is it returns the relative position on the grid
    double norm( T _value ) const;
    
    /// calculates the cell in which the value lies
    /** The cell representing values just higher than the base is cell 0.
     */
    int cellNr( T _value ) const;
    
  };
  
  /****************************//**
   * functions
   *****************************/
    
  TEMPT
  bool NumericIterator<T>::operator==( const NumericIterator<T>& _to_compare ) const
  {
    if( 
      base_!=_to_compare.base_ || 
      step_size_!=_to_compare.step_size_ ||
      cellNr(value_)!=cellNr(_to_compare.value_) 
    )
      return false;
    else
      return true;    
  }
  
  TEMPT
  bool NumericIterator<T>::operator!=( const NumericIterator<T>& _to_compare ) const
  {
    return !(*this==_to_compare);
  }
  
  TEMPT
  T& NumericIterator<T>::operator*()
  {
    return value_;
  }
  
  TEMPT
  T* NumericIterator<T>::operator->()
  {
    return &value_;
  }
  
  TEMPT
  NumericIterator<T>& NumericIterator<T>::operator++()
  {
    value_+=step_size_;
    return *this;
  }
  
  TEMPT
  NumericIterator<T> NumericIterator<T>::operator++(int)
  {
    NumericIterator<T> copy(*this);
    operator++();
    return copy;
  }
  
  TEMPT
  NumericIterator<T>& NumericIterator<T>::operator--()
  {
    value_-=step_size_;
    return *this;
  }
  
  TEMPT
  NumericIterator<T> NumericIterator<T>::operator--(int)
  {
    NumericIterator<T> copy(*this);
    operator--();
    return copy;
  }
  
  TEMPT
  NumericIterator<T> NumericIterator<T>::operator+(int _offset)
  {
    NumericIterator<T> new_it(*this);
    new_it.value_ += _offset*step_size_;
    return new_it;
  }
  
  TEMPT
  NumericIterator<T> NumericIterator<T>::operator-(int _offset)
  {
    NumericIterator<T> new_it(*this);
    new_it.value_ -= _offset*step_size_;
    return new_it;
  }
  
  TEMPT
  bool NumericIterator<T>::operator<( const NumericIterator<T>& _to_compare ) const
  {
    if( 
      base_!=_to_compare.base_ || 
      step_size_!=_to_compare.step_size_
    )
      return false;
    else
      return cellNr(value_) < cellNr(_to_compare.value_);    
  }
  
  TEMPT
  bool NumericIterator<T>::operator>( const NumericIterator<T>& _to_compare ) const
  {
    if( 
      base_!=_to_compare.base_ || 
      step_size_!=_to_compare.step_size_
    )
      return false;
    else
      return cellNr(value_) > cellNr(_to_compare.value_);    
  }
  
  TEMPT
  bool NumericIterator<T>::operator<=( const NumericIterator<T>& _to_compare ) const
  {
    if( 
      base_!=_to_compare.base_ || 
      step_size_!=_to_compare.step_size_
    )
      return false;
    else
      return cellNr(value_) <= cellNr(_to_compare.value_);    
  }
  
  TEMPT
  bool NumericIterator<T>::operator>=( const NumericIterator<T>& _to_compare ) const
  {
    if( 
      base_!=_to_compare.base_ || 
      step_size_!=_to_compare.step_size_
    )
      return false;
    else
      return cellNr(value_) >= cellNr(_to_compare.value_);    
  }
  
  TEMPT
  void NumericIterator<T>::operator+=( int _offset )
  {
    value_ += _offset*step_size_;
  }
  
  TEMPT
  void NumericIterator<T>::operator-=( int _offset )
  {
    value_ -= _offset*step_size_;
  }
  
  TEMPT
  T& NumericIterator<T>::operator[]( int _offset )
  {
    return value_; // just a dummy to fill in
  }
  
  TEMPT
  double NumericIterator<T>::norm( T _value ) const
  {
    return (_value-base_)/(double)step_size_;
  }
  
  TEMPT
  int NumericIterator<T>::cellNr( T _value ) const
  {
    double normed = norm(_value);
    if( normed>=0 )
      return (int)normed;
    else
      return (int)normed-1;
  }
}

#undef TEMPT