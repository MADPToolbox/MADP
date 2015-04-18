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


#ifndef __defined_libdai_exceptions_h
#define __defined_libdai_exceptions_h


#include <exception>
#include <stdexcept>
#include <string>


#define DAI_QUOTE(x) #x
#define DAI_TOSTRING(x) DAI_QUOTE(x)

#define DAI_THROW(cod) throw libDAI::Exception(libDAI::Exception::cod, std::string(__FILE__ ", line " DAI_TOSTRING(__LINE__)))


namespace libDAI {


    class Exception : public std::runtime_error {
        public:
            Exception(size_t code, const std::string& msg = "") : std::runtime_error(ErrorStrings[code] + " [" +  msg + "]") {}

        enum {NOT_IMPLEMENTED,
              UNKNOWN_DAI_ALGORITHM,
              UNKNOWN_PROPERTY_TYPE,
              MALFORMED_PROPERTY,
              UNKNOWN_ENUM_VALUE,
              CANNOT_READ_FILE,
              CANNOT_WRITE_FILE,
              INVALID_FACTORGRAPH_FILE,
              NOT_ALL_PROPERTIES_SPECIFIED,
              MULTIPLE_UNDO,
              FACTORGRAPH_NOT_CONNECTED,
              INTERNAL_ERROR,
              NUM_ERRORS};  // NUM_ERRORS should be the last entry

        private:
            static std::string ErrorStrings[NUM_ERRORS];
    };


}


#endif
