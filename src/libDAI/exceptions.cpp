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


#include "exceptions.h"


namespace libDAI {


    std::string Exception::ErrorStrings[NUM_ERRORS] = {
        "This feature is not yet implemented",
        "Unknown DAI algorithm",
        "Unknown Property type",
        "Malformed Property",
        "Unknown ENUM value",
        "Cannot read file",
        "Cannot write file",
        "Invalid FactorGraph file",
        "Not all mandatory Properties specified",
        "Multiple undo levels unsupported",
        "FactorGraph is not connected",
        "Internal error"
    }; 


}
