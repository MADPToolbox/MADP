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


#ifndef __defined_libdai_box_h
#define __defined_libdai_box_h


#include <iostream>
#include <algorithm>
#include <vector>
#include "daialg.h"
#include "weightedgraph.h"
#include "factorgraph.h"


//#define GLPK


namespace libDAI {


    class Box {
        private:
            VarSet _vars;
        public:
            Prob _min, _max;
        public:
            Box() : _vars(), _min(), _max() {}
            Box(const VarSet &ns) : _vars(ns), _min(ns.stateSpace(), 0.0), _max(ns.stateSpace(), 1.0) {}
            Box(const VarSet &ns, double mi, double ma) : _vars(ns), _min(ns.stateSpace(), mi), _max(ns.stateSpace(), ma) {
                assert(mi <= ma);
            }
            Box(const VarSet &ns, const Prob &mi, const Prob &ma) : _vars(ns), _min(mi), _max(ma) {
                assert(_min <= _max);
            }
            Box(const VarSet &ns, const std::vector<Prob> &points) : _vars(ns) {
                if( points.size() == 0 ) {
                    double x = 1.0 / ns.stateSpace();
                    _min = Prob(ns.stateSpace(), x);
                    _max = Prob(ns.stateSpace(), x);
                } else {
                    size_t N = ns.stateSpace();
                    assert( points[0].size() == N );
                    _min = _max = points[0];
                    for( size_t i = 1; i < points.size(); i++ ) {
                        assert( points[i].size() == N );
                        _min = libDAI::min( _min, points[i] );
                        _max = libDAI::max( _max, points[i] );
                    }
                }
            }
            Box & setFull() {
                _min.fill(0.0);
                _max.fill(1.0);
                return *this;
            }
            Box & setEmpty() {
                _min.fill(1.0);
                _max.fill(1.0);
                return *this;
            }
            Prob randomSample() {
                return Prob(_vars.stateSpace()).randomize() * (_max - _min) + _min;
            }
            Box( const Box & x ) : _vars(x._vars), _min(x._min), _max(x._max) {}
            Box & operator=( const Box & x ) {
                if( this != &x ) {
                    _vars = x._vars;
                    _min  = x._min;
                    _max  = x._max;
                }
                return *this;
            }

            friend std::ostream& operator<< (std::ostream & os, const Box& c) {
                os << "[" << c._vars << ":" << c._min << " <= " << c._max << "]";
                return( os );
            }
            const VarSet & vars() const { return _vars; }
            const Prob & min() const { return _min; }
            const Prob & max() const { return _max; }
            Prob & min() { return _min; }
            Prob & max() { return _max; }
            Box operator* (const Box &c) const {
//#ifdef DEBUG
                assert( _min <= _max );
                assert( c._min <= c._max );
//#endif
                assert( _vars == c._vars );
                return Box( _vars, _min * c._min, _max * c._max );
            }
            Box& operator*= (const Box &c) {
//#ifdef DEBUG
                assert( _min <= _max );
                assert( c._min <= c._max );
//#endif
                assert( _vars == c._vars );
                _min *= c._min;
                _max *= c._max;
                return *this;
            }
            std::vector<Prob> extremePoints() const {
                bool min_zero = false;
                if( _min.totalSum() < 1e-14 )
                    min_zero = true;
                bool min_is_max = false;
                if( _max.totalSum() - _min.totalSum() < 1e-14 )
                    min_is_max = true;

/*                if( min_is_max ) {
                    return std::vector<Prob>(1,_min);
                } else if( min_zero ) {
                    size_t N = _vars.stateSpace();
                    std::vector<Prob> result( N, Prob(N, 0.0) );
                    for( size_t i = 0; i < N; i++ )
                        result[i][i] = 1.0;
                    return result;
                } else {*/
                    size_t maxbits = sizeof(size_t) * 8;
                    size_t N = _vars.stateSpace();
                    assert( N < maxbits );
                    if( N > 16 )
                        std::cerr << "Warning: calculating " << (1UL << N) << " extreme points..." << std::endl;
                    std::vector<Prob> result;
                    result.reserve( 1UL << (N - (min_zero ? 1 : 0)) );

                    for( size_t l = (min_zero ? 1 : 0); l < (1UL << N); l++ ) {
                        Prob point(N, 0.0);
                        for( size_t i = 0; i < N; i++ )
                            if( l & (1UL << i) )
                                point[i] = _max[i];
                            else
                                point[i] = _min[i];
                        result.push_back( point );
                    }
                    return result;
//                }
            }
            Box normalize() {
                return( *this = normalized() );
            }
            Box normalized() const {
                std::vector<Prob> ep = extremePoints();
                for( size_t l = 0; l < ep.size(); l++ )
                    ep[l].normalize();
                return Box( _vars, ep );
            }
            Box boundSumProd(Factor psi, VarSet dest, bool normed) const {
                // FIXME: do we have to store the extreme points?
                std::vector<Prob> ep = extremePoints();
                Prob image_min( dest.stateSpace(), INFINITY );
                Prob image_max( dest.stateSpace(), -INFINITY );
                for( size_t l = 0; l < ep.size(); l++ ) {
                    // FIXME: this can be sped up massively by caching the index
                    Prob image_ep;
                    image_ep = (psi * Factor(_vars,ep[l])).marginal(dest,normed).p();
                    image_min = libDAI::min( image_ep, image_min );
                    image_max = libDAI::max( image_ep, image_max );
                }
                return Box( dest, image_min, image_max );
            }
            bool contains( Factor psi, double tol ) const {
                assert( psi.vars() == _vars );
                if( ((_min - tol) <= psi.p()) && (psi.p() <= (_max + tol)) )
                    return true;
                else
                    return false;
            }
            bool contains( Box box, double tol ) const {
                assert( box.vars() == _vars );
                if( ((_min - tol) <= box._min) && (box._max <= (_max + tol)) )
                    return true;
                else 
                    return false;
            }
            double gap() const {
                if( _max.hasNaNs() || _min.hasNaNs() )
                    return 1.0;
                else
                    return (_max - _min).maxVal();
            }
            friend Box boundSumProd( Factor psi, std::vector<Box> &incomingBoxes, VarSet dest, bool independent, bool normed );
            friend Box boundProd( std::vector<Box> &boxes );


            /// Test for equality (only compares _vars)
            friend bool operator==( const Box &a, const Box &b ) {
                return (a._vars == b._vars);
            }

            /// Test for inequality (only compares _vars)
            friend bool operator!=( const Box &a, const Box &b ) {
                return !(a._vars == b._vars);
            }

            /// Lexicographical comparison (only compares _vars)
            friend bool operator<( const Box &a, const Box &b ) {
                return a._vars < b._vars;
            }
    };


    double FactorStrength( const Factor &psi, const Var &i, const Var &j, const Box &hbox );
    double Ihler_recursion( const Factor & psi, double delta );
    double Ihler_distance( const Factor & psi1, const Factor & psi2 );
    Box Ihler_box( Var v_i, const Factor & psi_i, double delta );

#ifdef GLPK
    bool TestIfPointInConvexHull( std::vector<Prob> points, Prob point );
#endif


}


#endif
