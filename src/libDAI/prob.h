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


#ifndef __defined_libdai_prob_h
#define __defined_libdai_prob_h


#include <complex>
#include <cmath>
#include <vector>
#include <iostream>
#include <cassert>
#include "util.h"


namespace libDAI {


    typedef double                  Real;
    typedef std::complex<double>    Complex;

    template<typename T> class      TProb;


    // predefine friends
    template<typename T> TProb<T> min( const TProb<T> &a, const TProb<T> &b );
    template<typename T> TProb<T> max( const TProb<T> &a, const TProb<T> &b );


    /// TProb<T> implements a probability vector of type T.
    /// T should be castable from and to double and to complex.
    template <typename T> class TProb {
        protected:
            /// The entries
            std::vector<T> _p;

        private:
            /// Calculate x times log(x), or 0 if x == 0
            Complex xlogx( Real x ) const { return( x == 0.0 ? 0.0 : Complex(x) * std::log(Complex(x))); }

        public:
            /// NORMPROB means that the sum of all entries should be 1
            /// NORMLINF means that the maximum absolute value of all entries should be 1
            typedef enum { NORMPROB, NORMLINF } NormType;
            /// DISTL1 is the L-1 distance (sum of absolute values of pointwise difference)
            /// DISTLINF is the L-inf distance (maximum absolute value of pointwise difference)
            /// DISTTV is the Total Variation distance
            typedef enum { DISTL1, DISTLINF, DISTTV } DistType;
            
            /// Default constructor
            TProb() : _p() {}
            
            /// Construct uniform distribution of given length
            TProb( size_t n ) : _p(std::vector<T>(n, 1.0 / n)) {}
            
            /// Construct with given length and initial value
            TProb( size_t n, Real p ) : _p(std::vector<T>(n,(T)p)) {}
            
            /// Construct with given length and initial array
            TProb( size_t n, const Real* p ) {
                // Reserve-push_back is faster than resize-copy
                _p.reserve( n );
                for( size_t i = 0; i < n; i++ ) 
                    _p.push_back( p[i] );
            }
            
            /// Copy constructor
            TProb( const TProb<T> & x ) : _p(x._p) {}
            
            /// Assignment operator
            TProb<T> & operator=( const TProb<T> &x ) {
                if( this != &x ) {
                    _p = x._p;
                }
                return *this;
            }
            
            /// Provide read access to _p
            const std::vector<T> & p() const { return _p; }

            /// Provide full access to _p
            std::vector<T> & p() { return _p; }
            
            /// Provide read access to ith element of _p
            T operator[]( size_t i ) const { return _p[i]; }
            
            /// Provide full access to ith element of _p
            T& operator[]( size_t i ) { return _p[i]; }

            /// Set all elements to x
            TProb<T> & fill(T x) { 
                for( size_t i = 0; i < size(); i++ )
                    _p[i] = x;
                return *this;
            }

            /// Set all elements to iid random numbers from uniform(0,1) distribution
            TProb<T> & randomize() { 
                for( size_t i = 0; i < size(); i++ )
                    _p[i] = rnd_uniform();
                return *this;
            }

            /// Return size
            size_t size() const {
                return _p.size();
            }

            /// Make entries zero if (Real) absolute value smaller than epsilon
            TProb<T>& makeZero (Real epsilon) {
                for( size_t i = 0; i < size(); i++ )
                    if( fabs((Real)_p[i]) < epsilon )
                        _p[i] = 0;
                return *this;
            }

            /// Make entries epsilon if they are smaller than epsilon
            TProb<T>& makePositive (Real epsilon) {
                for( size_t i = 0; i < size(); i++ )
                    if( (0 < (Real)_p[i]) && ((Real)_p[i] < epsilon) )
                        _p[i] = epsilon;
                return *this;
            }

            /// Multiplication with T x
            TProb<T>& operator*= (T x) {
                for( size_t i = 0; i < size(); i++ )
                    _p[i] *= x;
                return *this;
            }

            /// Return product of *this with T x
            TProb<T> operator* (T x) const {
                TProb<T> prod( *this );
                prod *= x;
                return prod;
            }

            /// Division by T x
            TProb<T>& operator/= (T x) {
#ifdef DEBUG
                assert( x != 0.0 );
#endif
                for( size_t i = 0; i < size(); i++ )
                    _p[i] /= x;
                return *this;
            }

            /// Return quotient of *this and T x
            TProb<T> operator/ (T x) const {
                TProb<T> prod( *this );
                prod /= x;
                return prod;
            }

            /// Pointwise comparison
            bool operator<= (const TProb<T> & q) const {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ )
                    if( !(_p[i] <= q[i]) )
                        return false;
                return true;
            }

            /// Pointwise multiplication with q
            TProb<T>& operator*= (const TProb<T> & q) {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ )
                    _p[i] *= q[i];
                return *this;
            }
            
            /// Return product of *this with q
            TProb<T> operator* (const TProb<T> & q) const {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                TProb<T> prod( *this );
                prod *= q;
                return prod;
            }

            /// Pointwise addition with q
            TProb<T>& operator+= (const TProb<T> & q) {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ )
                    _p[i] += q[i];
                return *this;
            }
            
            /// Pointwise subtraction of q
            TProb<T>& operator-= (const TProb<T> & q) {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ )
                    _p[i] -= q[i];
                return *this;
            }
            
            /// Pointwise addition with q
            TProb<T>& operator+= (T q) {
                for( size_t i = 0; i < size(); i++ )
                    _p[i] += q;
                return *this;
            }
            
            /// Pointwise subtraction of q
            TProb<T>& operator-= (T q) {
                for( size_t i = 0; i < size(); i++ )
                    _p[i] -= q;
                return *this;
            }
            
            /// Return sum of *this and q
            TProb<T> operator+ (T q) const {
                TProb<T> sum( *this );
                sum += q;
                return sum;
            }
            
            /// Return *this minus q
            TProb<T> operator- (T q) const {
                TProb<T> sum( *this );
                sum -= q;
                return sum;
            }

            /// Return sum of *this and q
            TProb<T> operator+ (const TProb<T> & q) const {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                TProb<T> sum( *this );
                sum += q;
                return sum;
            }
            
            /// Return *this minus q
            TProb<T> operator- (const TProb<T> & q) const {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                TProb<T> sum( *this );
                sum -= q;
                return sum;
            }

            /// Pointwise division by q
            TProb<T>& operator/= (const TProb<T> & q) {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ ) {
#ifdef DEBUG
    //              assert( q[i] != 0.0 );
#endif
                    if( q[i] == 0.0 )       // FIXME
                        _p[i] = 0.0;
                    else
                        _p[i] /= q[i];
                }
                return *this;
            }
            
            /// Pointwise division by q, division by zero yields infinity
            TProb<T>& divide (const TProb<T> & q) {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                for( size_t i = 0; i < size(); i++ )
                    _p[i] /= q[i];
                return *this;
            }
            
            /// Return quotient of *this with q
            TProb<T> operator/ (const TProb<T> & q) const {
#ifdef DEBUG
                assert( size() == q.size() );
#endif
                TProb<T> quot( *this );
                quot /= q;
                return quot;
            }

            /// Return pointwise inverse
            TProb<T> inverse(bool zero = false) const {
                TProb<T> inv;
                inv._p.reserve( size() );
                if( zero )
                    for( size_t i = 0; i < size(); i++ )
                        inv._p.push_back( _p[i] == 0.0 ? 0.0 : 1.0 / _p[i] );
                else
                    for( size_t i = 0; i < size(); i++ ) {
#ifdef DEBUG
                        assert( _p[i] != 0.0 );
#endif
                        inv._p.push_back( 1.0 / _p[i] );
                    }
                return inv;
            }

            /// Return *this to the power of a (pointwise)
            TProb<T>& operator^= (Real a) {
                if( a != 1.0 ) {
                    for( size_t i = 0; i < size(); i++ )
                        _p[i] = std::pow( _p[i], a );
                }
                return *this;
            }

            /// Pointwise power of a
            TProb<T> operator^ (Real a) const {
                TProb<T> power;
                if( a != 1.0 ) {
                    power._p.reserve( size() );
                    for( size_t i = 0; i < size(); i++ )
                        power._p.push_back( std::pow( _p[i], a ) );
                } else
                    power = *this;
                return power;
            }

            /// Pointwise signum
            TProb<T> sgn() const {
                TProb<T> x;
                x._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ ) {
                    T s = 0;
                    if( _p[i] > 0 )
                        s = 1;
                    else if( _p[i] < 0 )
                        s = -1;
                    x._p.push_back( s );
                }
                return x;
            }

            TProb<T> abs() const {
                TProb<T> x;
                x._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    x._p.push_back( std::fabs( _p[i] ) );
                return x;
            }

            /// Pointwise exp
            TProb<T> exp() const {
                TProb<T> e;
                e._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    e._p.push_back( std::exp( _p[i] ) );
                return e;
            }

            /// Pointwise log
            TProb<T> log() const {
                TProb<T> l;
                l._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    l._p.push_back( std::log( _p[i] ) );
                return l;
            }

            /// Pointwise log (or 0 if == 0)
            TProb<T> log0() const {
                TProb<T> l0;
                l0._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    l0._p.push_back( (_p[i] == 0.0) ? 0.0 : std::log( _p[i] ) );
                return l0;
            }

            /// Pointwise (complex) log (or 0 if == 0)
    /*      CProb clog0() const {
                CProb l0;
                l0._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    l0._p.push_back( (_p[i] == 0.0) ? 0.0 : std::log( Complex( _p[i] ) ) );
                return l0;
            }*/

            /// Return distance of p and q
            friend Real dist( const TProb<T> & p, const TProb<T> & q, DistType dt ) {
#ifdef DEBUG
                assert( p.size() == q.size() );
#endif
                Real result = 0.0;
                switch( dt ) {
                    case DISTL1:
                        for( size_t i = 0; i < p.size(); i++ )
                            result += fabs((Real)p[i] - (Real)q[i]);
                        break;
                        
                    case DISTLINF:
                        for( size_t i = 0; i < p.size(); i++ ) {
                            Real z = fabs((Real)p[i] - (Real)q[i]);
                            if( z > result )
                                result = z;
                        }
                        break;

                    case DISTTV:
                        for( size_t i = 0; i < p.size(); i++ )
                            result += fabs((Real)p[i] - (Real)q[i]);
                        result *= 0.5;
                        break;
                }
                return result;
            }

            /// Return (complex) Kullback-Leibler distance with q
            friend Complex KL_dist( const TProb<T> & p, const TProb<T> & q ) {
#ifdef DEBUG
                assert( p.size() == q.size() );
#endif
                Complex result = 0.0;
                for( size_t i = 0; i < p.size(); i++ ) {
                    if( (Real) p[i] != 0.0 ) {
                        Complex p_i = p[i];
                        Complex q_i = q[i];
                        result += p_i * (std::log(p_i) - std::log(q_i));
                    }
                }
                return result;
            }

            /// Return sum of all entries
            T totalSum() const {
                T Z = 0.0;
                for( size_t i = 0; i < size(); i++ )
                    Z += _p[i];
                return Z;
            }

            /// Converts entries to Real and returns maximum absolute value
            T maxAbs() const {
                T Z = 0.0;
                for( size_t i = 0; i < size(); i++ ) {
                    Real mag = fabs( (Real) _p[i] );
                    if( mag > Z )
                        Z = mag;
                }
                return Z;
            }

            /// Returns maximum value
            T maxVal() const {
                T Z = 0.0;
                for( size_t i = 0; i < size(); i++ ) {
                    if( _p[i] > Z )
                        Z = _p[i];
                }
                return Z;
            }

            /// Returns minimum value
            T minVal() const {
                assert( size() > 0 );
                T Z = _p[0];
                for( size_t i = 0; i < size(); i++ ) {
                    if( _p[i] < Z )
                        Z = _p[i];
                }
                return Z;
            }

            /// Normalize, using the specified norm
            T normalize( NormType norm = NORMPROB ) {
                T Z = 0.0;
                if( norm == NORMPROB )
                    Z = totalSum();
                else if( norm == NORMLINF )
                    Z = maxAbs();
#ifdef DEBUG
                assert( Z != 0.0 );
#endif
                T Zi = 1.0 / Z;
                for( size_t i = 0; i < size(); i++ )
                   _p[i] *= Zi;
                return Z;
            }

            /// Return normalized copy of *this, using the specified norm
            TProb<T> normalized( NormType norm = NORMPROB ) const {
                T Z = 0.0;
                if( norm == NORMPROB ) 
                    Z = totalSum();
                else if( norm == NORMLINF )
                    Z = maxAbs();
#ifdef DEBUG
                assert( Z != 0.0 );
#endif
                Z = 1.0 / Z;
                
                TProb<T> result;
                result._p.reserve( size() );
                for( size_t i = 0; i < size(); i++ )
                    result._p.push_back( _p[i] * Z );
                return result;
            }
        
            /// Returns true if one or more entries are NaN
            bool hasNaNs() const {
                bool NaNs = false;
                for( size_t i = 0; i < size() && !NaNs; i++ ) 
                    if( isnan( _p[i] ) )
                        NaNs = true;
                return NaNs;
            }

            /// Returns true if one or more entries are negative
            bool hasNegatives() const {
                bool Negatives = false;
                for( size_t i = 0; i < size() && !Negatives; i++ ) 
                    if( _p[i] < 0.0 )
                        Negatives = true;
                return Negatives;
            }

            /// Returns (complex) entropy
            Complex entropy() const {
                Complex S = 0.0;
                for( size_t i = 0; i < size(); i++ )
                    S -= xlogx(_p[i]);
                return S;
            }

            /// Returns TProb<T> containing the pointwise minimum of a and b (which should have equal size)
            friend TProb<T> min <> ( const TProb<T> &a, const TProb<T> &b );

            /// Returns TProb<T> containing the pointwise maximum of a and b (which should have equal size)
            friend TProb<T> max <> ( const TProb<T> &a, const TProb<T> &b );

            friend std::ostream& operator<< (std::ostream& os, const TProb<T>& P) {
                os << "[";
                for( size_t i = 0; i < P.size(); i++ )
                    os << P._p[i] << " ";
                os << "]";
                return os;
            }
    };


    template<typename T> TProb<T> min( const TProb<T> &a, const TProb<T> &b ) {
        assert( a.size() == b.size() );
        TProb<T> result( a.size() );
        for( size_t i = 0; i < a.size(); i++ )
            if( a[i] < b[i] )
                result[i] = a[i];
            else
                result[i] = b[i];
        return result;
    }


    template<typename T> TProb<T> max( const TProb<T> &a, const TProb<T> &b ) {
        assert( a.size() == b.size() );
        TProb<T> result( a.size() );
        for( size_t i = 0; i < a.size(); i++ )
            if( a[i] > b[i] )
                result[i] = a[i];
            else
                result[i] = b[i];
        return result;
    }


    typedef TProb<Real>             Prob;
    typedef TProb<Complex>          CProb;


}


#endif
