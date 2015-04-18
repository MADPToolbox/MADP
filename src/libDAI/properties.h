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


#ifndef __defined_libdai_properties_h
#define __defined_libdai_properties_h


#include <iostream>
#include <sstream>
#include <boost/any.hpp>
#include <map>
#include <cassert>
#include <typeinfo>


namespace libDAI {


    typedef std::string PropertyKey;
    typedef boost::any  PropertyValue;
    typedef std::pair<PropertyKey, PropertyValue> Property;


    /// Sends a Properties object to an output stream
    std::ostream& operator<< (std::ostream & os, const Property & p);


    /// The Properties class represents a set of properties
    class Properties : public std::map<PropertyKey, PropertyValue> {
        public:
            /// Gets a property
            const PropertyValue & Get(const PropertyKey &key) const { 
                Properties::const_iterator x = find(key); 
                assert( x != this->end() ); 
                return x->second; 
            }

            /// Sets a property 
            Properties & Set(const PropertyKey &key, const PropertyValue &val) { this->operator[](key) = val; return *this; }

            /// Gets a property, casted as ValueType
            template<typename ValueType>
            ValueType GetAs(const PropertyKey &key) const {
                try {
                    return boost::any_cast<ValueType>(Get(key));
                } catch( const boost::bad_any_cast & ) {
                    std::cerr << "Cannot cast property " << key << " to ";
                    std::cerr << typeid(ValueType).name() << std::endl;
                    return boost::any_cast<ValueType>(Get(key));
                }
            }

            /// Converts a property from string to ValueType, if necessary
            template<typename ValueType>
            void ConvertTo(const PropertyKey &key) { 
                PropertyValue val = Get(key);
                if( val.type() != typeid(ValueType) ) {
                    assert( val.type() == typeid(std::string) );

                    std::stringstream ss;
                    ss << GetAs<std::string>(key);
                    ValueType result;
                    ss >> result;

                    Set(key, result);
                }
            }

            /// Converts a property from string to ValueType, if necessary
            template<typename ValueType>
            ValueType FromStringTo(const PropertyKey &key) const { 
                PropertyValue val = Get(key);
                if( val.type() != typeid(ValueType) ) {
                    assert( val.type() == typeid(std::string) );

                    std::stringstream ss;
                    ss << GetAs<std::string>(key);
                    ValueType result;
                    ss >> result;

                    return result;
                } else
                    return GetAs<ValueType>(key);
            }

            /// Shorthand for (temporarily) adding properties, e.g. Properties p()("method","BP")("verbose",1)("tol",1e-9)
            Properties operator()(const PropertyKey &key, const PropertyValue &val) const { Properties copy = *this; return copy.Set(key,val); }

            /// Check if a property with given key exists
            bool hasKey(const PropertyKey &key) const { Properties::const_iterator x = find(key); return (x != this->end()); }

            /// Sends a Properties object to an output stream
            friend std::ostream& operator<< (std::ostream & os, const Properties & ps);

            /// Reads a Properties object from an input stream
            friend std::istream& operator >> (std::istream& is, Properties & ps);
    };


}


#endif
