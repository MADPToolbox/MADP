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


#ifndef __defined_libdai_util_h
#define __defined_libdai_util_h


#include <sys/times.h>
#include <vector>
#include <set>
#include <iostream>


namespace libDAI {


    clock_t toc();
    void rnd_seed( size_t seed );
    double rnd_uniform();
    double rnd_stdnormal();
    int rnd_int( int min, int max );

    // Output a vector
    template<class T> 
    std::ostream& operator << (std::ostream& os, const std::vector<T> & x) {
        os << "(";
        for( typename std::vector<T>::const_iterator it = x.begin(); it != x.end(); it++ )
            os << (it != x.begin() ? ", " : "") << *it;
        os << ")";
        return os;
    }

    // Output a set
    template<class T> 
    std::ostream& operator << (std::ostream& os, const std::set<T> & x) {
        os << "{";
        for( typename std::set<T>::const_iterator it = x.begin(); it != x.end(); it++ )
            os << (it != x.begin() ? ", " : "") << *it;
        os << "}";
        return os;
    }

}


#endif
