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


#ifndef __defined_libdai_index_h
#define __defined_libdai_index_h


#include <vector>
#include "varset.h"


namespace libDAI {


/* Example:
 *
 * Index i ({s_j_1,s_j_2,...,s_j_m}, {s_1,...,s_N});    // j_k in {1,...,N}
 * for( ; i>=0; ++i ) {
 *      // loops over all states of (s_1,...,s_N)
 *      // i is linear index of corresponding state of (s_j_1, ..., s_j_m)
 * }
 */
    class Index
    {
    private:
        long _index;
        std::vector<int> _count,_max,_sum;
    public:
        Index () { _index=-1; };
        Index (const VarSet& P, const VarSet& ns)
        {
            long sum=1;
            VarSet::const_iterator j=ns.begin();
            for(VarSet::const_iterator i=P.begin();i!=P.end();++i)
            {//for the next i in P:

                for(;j!=ns.end()&&j->label()<=i->label();++j)
                {
                    //for the next j in ns, that has j <= i

                    _count.push_back(0);
                    _max.push_back(j->states());
                    _sum.push_back((i->label()==j->label())?sum:0);
                };
                sum*=i->states();
            };
            //we may not have walked over all j in ns yet, so finish:
            for(;j!=ns.end();++j)
            {
                _count.push_back(0);
                _max.push_back(j->states());
                _sum.push_back(0);
            };
            _index=0;
        };
        Index (const Index & ind) : _index(ind._index), _count(ind._count), _max(ind._max), _sum(ind._sum) {};
        Index & operator=(const Index & ind) {
            if(this!=&ind) {
                _index = ind._index;
                _count = ind._count;
                _max = ind._max;
                _sum = ind._sum;
            }
            return *this;
        }
        Index& clear ()
        {
            for(unsigned i=0;i!=_count.size();++i) _count[i]=0;
            _index=0;
            return(*this);
        };
        operator long () const { return(_index); };
        Index& operator ++ ()
        {
            if(_index>=0)
            {
                unsigned i;
                for(i=0;(i<_count.size())
                        &&(_index+=_sum[i],++_count[i]==_max[i]);++i)
                {
                    _index-=_sum[i]*_max[i];
                    _count[i]=0;
                };
                if(i==_count.size()) _index=-1;
            };
            return(*this);
        };
    };
    
    /** FRANS: multind performs joint <-> individual indices conversions
     * I think?!
     */
    class multind {
        private:
            std::vector<size_t> _dims;  // dimensions
            std::vector<size_t> _pdims; // products of dimensions

        public:
            multind(const std::vector<size_t> di) {
                _dims = di;
                size_t prod = 1;
                for( std::vector<size_t>::const_iterator i=di.begin(); i!=di.end(); i++ ) {
                    _pdims.push_back(prod);
                    prod = prod * (*i);
                }
                _pdims.push_back(prod);
            }
            multind(const VarSet& ns) {
                _dims.reserve( ns.size() ); 
                _pdims.reserve( ns.size() + 1 ); 
                size_t prod = 1;
                for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) {
                    _pdims.push_back( prod );
                    prod *= n->states();
                    _dims.push_back( n->states() );
                }
                _pdims.push_back( prod );
            }
            std::vector<size_t> vi(size_t li) const {   // linear index to vector index
                std::vector<size_t> v(_dims.size(),0);
                assert(li < _pdims.back());
                for( long j = v.size()-1; j >= 0; j-- ) {
                    size_t q = li / _pdims[j];
                    v[j] = q;
                    li = li - q * _pdims[j];
                }
                return v;
            }
            size_t li(const std::vector<size_t> vi) const { // linear index
                size_t s = 0;
                assert(vi.size() == _dims.size());
                for( size_t j = 0; j < vi.size(); j++ ) 
                    s += vi[j] * _pdims[j];
                return s;
            }
            size_t max() const { return( _pdims.back() ); };

            // FIXME add an iterator, which increases a vector index just using addition
    };

}


#endif
