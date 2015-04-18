/*  Copyright (C) 2006,2007  Joris Mooij  [joris at jorismooij dot nl]
    Radboud University Nijmegen, The Netherlands
    
    This file is part of libDAI.

    libDAI is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    libDAI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libDAI; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef __defined_libdai_var_h
#define __defined_libdai_var_h

#include <iostream>


namespace libDAI {


    /// Represents a discrete variable
    class Var {
        private:
            /// Internal label of the variable
            long    _label;

            /// Number of possible values
            size_t  _states;
            
        public:
            /// Default constructor
            Var() : _label(-1), _states(0) {};
            /// Constructor
            Var(long label, size_t states) : _label(label), _states(states) {};

            /// Read access to label
            long label() const { return _label; };
            /// Access to label
            long & label() { return _label; };

            /// Read access to states
            size_t states () const { return _states; };
            /// Access to states
            size_t& states () { return _states; };

            /// Smaller-than operator (compares labels)
            bool operator <  (const Var& n) const { return( _label <  n._label ); };
            /// Larger-than operator (compares labels)
            bool operator >  (const Var& n) const { return( _label >  n._label ); };
            /// Smaller-than-or-equal-to operator (compares labels)
            bool operator <= (const Var& n) const { return( _label <= n._label ); };
            /// Larger-than-or-equal-to operator (compares labels)
            bool operator >= (const Var& n) const { return( _label >= n._label ); };
            /// Not-equal-to operator (compares labels)
            bool operator != (const Var& n) const { return( _label != n._label ); };
            /// Equal-to operator (compares labels)
            bool operator == (const Var& n) const { return( _label == n._label ); };

            /// Stream output operator
            friend std::ostream& operator << (std::ostream& os, const Var& n) {
                return( os << "[" << n.label() << "]" );
            };
    };


}


#endif
