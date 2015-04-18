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


#ifndef __defined_libdai_boxbp_h
#define __defined_libdai_boxbp_h


#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"
#include "box.h"


namespace libDAI {


    class BoxBP {
        public:
            std::vector<Box> _boxes;

        public:
            /// Default constructor
            BoxBP() : _boxes() {};
            
            /// Get box on message from I to i
            const Box & box( const FactorGraph &fg, size_t i, size_t I ) const { return( _boxes[fg.edge(i,I)] ); }

            /// The actual algorithm
            void run( const FactorGraph &fg, size_t maxiter );

            /// Local calculation
            void calcNewBox( const FactorGraph &fg, size_t iI);
    };


}


#endif
