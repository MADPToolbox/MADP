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


#ifndef __defined_libdai_varset_h
#define __defined_libdai_varset_h


#include <vector>
#include <set>
#include <algorithm>
#if 0 // isn't found on MacOSX, necessary?
#include <ext/algorithm>
#endif
#include <iostream>
#include <cassert>
#include "var.h"


namespace libDAI {


    /// VarSet represents a set of variables; it is implemented as an ordered
    /// vector<Var> for efficiency reasons. In addition, it provides an easy 
    /// interface for set-theoretic operations by operator overloading.
    class VarSet {
        protected:
            /// The variables in this set
            std::vector<Var> _vars;

            /// Product of number of states of all contained variables
            size_t _statespace;

            /// Check whether ns is a subset
            bool includes( const VarSet& ns ) const {
                return std::includes( _vars.begin(), _vars.end(), ns._vars.begin(), ns._vars.end() );
            }

            /// Calculate statespace
            size_t calcStateSpace() {
                _statespace = 1;
                for( std::vector<Var>::const_iterator i = _vars.begin(); i != _vars.end(); ++i )
                    _statespace *= i->states();
                return _statespace;
            }


        public:
            /// Default constructor
            VarSet() : _vars(), _statespace(1) {};

            /// Construct a VarSet with one variable
            VarSet( const Var &n ) : _vars(), _statespace( n.states() ) { 
                _vars.push_back( n );
            }

            /// Construct a VarSet with two variables
            VarSet( const Var &n1, const Var &n2 ) { 
                if( n1 < n2 ) {
                    _vars.push_back( n1 );
                    _vars.push_back( n2 );
                } else if( n1 > n2 ) {
                    _vars.push_back( n2 );
                    _vars.push_back( n1 );
                } else
                    _vars.push_back( n1 );
                calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
            }

            /// Construct from a set<Var>
            VarSet( const std::set<Var> &ns ) {
                _vars.reserve( ns.size() );
                _vars.insert( _vars.begin(), ns.begin(), ns.end() );
                calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
            }

            /// Construct from a vector<Var>
            VarSet( const std::vector<Var> &ns ) {
                _vars.reserve( ns.size() );
                _vars.insert( _vars.begin(), ns.begin(), ns.end() );
                std::sort( _vars.begin(), _vars.end() );
                std::vector<Var>::iterator new_end = std::unique( _vars.begin(), _vars.end() );
                _vars.erase( new_end, _vars.end() );
                calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
            }

            /// Copy constructor
            VarSet( const VarSet &x ) : _vars( x._vars ), _statespace( x._statespace ) {}

            /// Assignment operator
            VarSet & operator=( const VarSet &x ) {
                if( this != &x ) {
                    _vars = x._vars;
                    _statespace = x._statespace;
#ifdef DEBUG
                    assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
                }
                return *this;
            }
            

            /// Return statespace, i.e. the product of the number of states of each variable
            size_t stateSpace() const { 
#ifdef DEBUG
                size_t x = 1;
                for( std::vector<Var>::const_iterator i = _vars.begin(); i != _vars.end(); i++ )
                    x *= i->states();
                assert( x == _statespace );
#endif
                return _statespace; 
            }
            

            /// Erase one variable
            VarSet& operator/= ( const Var& n ) { 
                std::vector<Var>::iterator pos = lower_bound( _vars.begin(), _vars.end(), n );
                if( pos != _vars.end() )
                    if( *pos == n ) { // found variable, delete it
                        _vars.erase( pos ); 
                        _statespace /= n.states();
                    }
#ifdef DEBUG
                assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
                return *this; 
            }

            /// Add one variable
            VarSet& operator|= ( const Var& n ) {
                std::vector<Var>::iterator pos = lower_bound( _vars.begin(), _vars.end(), n );
                if( pos == _vars.end() || *pos != n ) { // insert it
                    _vars.insert( pos, n );
                    _statespace *= n.states();
                }
#ifdef DEBUG
                assert( is_sorted( _vars.begin(), _vars.end() ) );
#endif
                return *this;
            }

            /// Setminus operator (result contains all variables except those in ns)
            VarSet operator/ (const VarSet& ns) const {
                VarSet res;
                std::set_difference( _vars.begin(), _vars.end(), ns._vars.begin(), ns._vars.end(), inserter( res._vars, res._vars.begin() ) );
                res.calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( res._vars.begin(), res._vars.end() ) );
#endif
                return res;
            }

            /// Set-union operator (result contains all variables plus those in ns)
            VarSet operator| (const VarSet& ns) const {
                VarSet res;
                std::set_union( _vars.begin(), _vars.end(), ns._vars.begin(), ns._vars.end(), inserter( res._vars, res._vars.begin() ) );
                res.calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( res._vars.begin(), res._vars.end() ) );
#endif
                return res;
            }

            /// Set-intersection operator (result contains all variables that are also contained in ns)
            VarSet operator& (const VarSet& ns) const {
                VarSet res;
                std::set_intersection( _vars.begin(), _vars.end(), ns._vars.begin(), ns._vars.end(), inserter( res._vars, res._vars.begin() ) );
                res.calcStateSpace();
#ifdef DEBUG
                assert( is_sorted( res._vars.begin(), res._vars.end() ) );
#endif
                return res;
            }
            
            /// Erases from *this all variables in ns
            VarSet& operator/= (const VarSet& ns) {
                return (*this = (*this / ns));
            }

            /// Adds to *this all variables in ns
            VarSet& operator|= (const VarSet& ns) {
                return (*this = (*this | ns));
            }

            /// Erases from *this all variables not in ns
            VarSet& operator&= (const VarSet& ns) { 
                return (*this = (*this & ns)); 
            }
            

            /// Returns true if *this and ns contain common variables
            bool operator&& (const VarSet& ns) const { 
                return !( (*this & ns).empty() ); 
            }

            /// Returns true if *this is a subset of ns
            bool operator<< (const VarSet& ns) const { 
                return ns.includes( *this ); 
            }

            /// Returns true if ns is a subset of *this
            bool operator>> (const VarSet& ns) const { 
                return includes( ns ); 
            }

            /// Returns true if *this contains the variable n
            bool operator&& (const Var& n) const { 
                return std::binary_search( _vars.begin(), _vars.end(), n );
            }

            
            /// Sends a VarSet to an output stream
            friend std::ostream& operator<< (std::ostream & os, const VarSet& ns) {
                for( std::vector<Var>::const_iterator n = ns._vars.begin(); n != ns._vars.end(); n++ )
                    os << *n;
                return( os );
            }


            // Constant iterator
            typedef std::vector<Var>::const_iterator const_iterator;
            /// Iterator
            typedef std::vector<Var>::iterator iterator;
            

            /// Returns iterator that points to the first variable
            iterator begin() { return _vars.begin(); }
            /// Returns constant iterator that points to the first variable
            const_iterator begin() const { return _vars.begin(); }

            /// Returns iterator that points beyond the last variable
            iterator end() { return _vars.end(); }
            /// Returns constant iterator that points beyond the last variable
            const_iterator end() const { return _vars.end(); }


            /// Returns number of variables
            std::vector<Var>::size_type size() const { return _vars.size(); }


            /// Returns whether the set is empty
            bool empty() const { return _vars.size() == 0; }


            /// Test for equality (ignores _statespace member)
            friend bool operator==( const VarSet &a, const VarSet &b ) {
                return (a._vars == b._vars);
            }

            /// Test for inequality (ignores _statespace member)
            friend bool operator!=( const VarSet &a, const VarSet &b ) {
                return !(a._vars == b._vars);
            }

            /// Lexicographical comparison (ignores _statespace member)
            friend bool operator<( const VarSet &a, const VarSet &b ) {
                return a._vars < b._vars;
            }
    };


    /// For two Vars n1 and n2, the expression n1 | n2 gives the Varset containing n1 and n2
    inline VarSet operator| (const Var& n1, const Var& n2) {
        return( VarSet(n1, n2) );
    }


}


#endif
