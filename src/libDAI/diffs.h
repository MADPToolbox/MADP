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


#ifndef __defined_libdai_diffs_h
#define __defined_libdai_diffs_h


#include <vector>


namespace libDAI {


    ///Frans: Diffs is a vector of doubles meant specifically to contain the difference between the probabilities assigned to each variable by the new belief and those of the old belief (?)
    class Diffs : public std::vector<double> {
        private:
            size_t _maxsize;
            double _def;
            std::vector<double>::iterator _pos;
            std::vector<double>::iterator _maxpos;
        public:
            Diffs(long maxsize, double def) : std::vector<double>(), _maxsize(maxsize), _def(def) { 
                this->reserve(_maxsize); 
                _pos = begin(); 
                _maxpos = begin(); 
            };
            double max() { 
                if( size() < _maxsize )
                    return _def;
                else
                    return( *_maxpos ); 
            }
            void push(double x) {
                if( size() < _maxsize ) {
                    push_back(x);
                    _pos = end();
                    if( size() > 1 ) {
                        if( *_maxpos < back() ) {
                            _maxpos = end();
                            _maxpos--;
                        }
                    } else {
                        _maxpos = begin();
                    }
                }
                else {
                    if( _pos == end() )
                        _pos = begin();
                    if( _maxpos == _pos ) {
                        *_pos++ = x; 
                        _maxpos = max_element(begin(),end());
                    } else {
                        if( x > *_maxpos )
                            _maxpos = _pos;
                        *_pos++ = x;
                    }
                }
            }
            size_t max_size() { return _maxsize; }
    };


}


#endif
