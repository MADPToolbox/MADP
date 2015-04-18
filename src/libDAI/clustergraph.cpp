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


#include <set>
#include <vector>
#include <iostream>
#include "varset.h"
#include "clustergraph.h"


namespace libDAI {


    using namespace std;


    /// Construct from vector<VarSet>
    ClusterGraph::ClusterGraph( const std::vector<VarSet> & cls ) : BC() {
        // Add clusters as nodes of type 2, excluding duplicates
        VarSet vars;
        for( std::vector<VarSet>::const_iterator cl = cls.begin(); cl != cls.end(); cl++ )
            if( find( V2s().begin(), V2s().end(), *cl ) == V2s().end() ) {
                V2s().push_back( *cl );
                vars |= *cl;
            }

        // Add variables as nodes of type 1
        V1s().reserve( vars.size() );
        for( VarSet::const_iterator n = vars.begin(); n != vars.end(); n++ )
            V1s().push_back( *n );

        // Create edges
        for( size_t i2 = 0; i2 < nr2(); i2++ ) {
            VarSet & ns = V2(i2);
            for( VarSet::const_iterator q = ns.begin(); q != ns.end(); q++ ) {
                for( size_t i1 = 0; i1 < nr1(); i1++ ) {
                    if( V1(i1) == *q ) {
                        edges().push_back(edge_type(i1,i2));
                        break;
                    }
                }
            }
        }

        // Calc neighbours and adjacency matrix
        Regenerate();
    }

    
    ClusterGraph ClusterGraph::VarElim_MinFill() const {
        // Make a copy
        ClusterGraph Cl(*this);
        Cl.eraseNonMaximal();

        ClusterGraph result;
        VarSet vs( vars() );
        
        // Do variable elimination
        while( !vs.empty() ) {
            VarSet::const_iterator lowest = vs.end();
            size_t lowest_cost = -1UL;
            for( VarSet::const_iterator n = vs.begin(); n != vs.end(); n++ ) {
                size_t cost = Cl.eliminationCost( *n );
                if( lowest == vs.end() || lowest_cost > cost ) {
                    lowest = n;
                    lowest_cost = cost;
                }
            }
            Var n = *lowest;

            result.insert( Cl.Delta(n) );

            Cl.insert( Cl.delta(n) );
            Cl.eraseSubsuming( n );
            Cl.eraseNonMaximal();
            vs /= n;

        }

        return result;
    }


    /// As Taylan does it, but return the elimination sequence instead of resulting ClusterGraph
    vector<Var> ClusterGraph::MinFill() const {
        // Make a copy
        ClusterGraph Cl(*this);
        Cl.eraseNonMaximal();

        vector<Var> result;
        VarSet _vars( vars() );
        
        // Do variable elimination
        while( !_vars.empty() ) {
            VarSet::const_iterator lowest = _vars.end();
            size_t lowest_cost = -1UL;
            for( VarSet::const_iterator n = _vars.begin(); n != _vars.end(); n++ ) {
                size_t cost = Cl.eliminationCost( *n );
                if( lowest == _vars.end() || lowest_cost > cost ) {
                    lowest = n;
                    lowest_cost = cost;
                }
            }
            Var n = *lowest;

            result.push_back( n );

            Cl.insert( Cl.delta(n) );
            Cl.eraseSubsuming( n );
            Cl.eraseNonMaximal();
            _vars /= n;

        }

        return result;
    }


    ClusterGraph ClusterGraph::VarElim( const std::vector<Var> & ElimSeq ) const {
        // Make a copy
        ClusterGraph Cl(*this);

        ClusterGraph result;
        Cl.eraseNonMaximal();
        
        // Do variable elimination
        for( vector<Var>::const_iterator n = ElimSeq.begin(); n != ElimSeq.end(); n++ ) {
            assert( Cl.vars() && *n );

            result.insert( Cl.Delta(*n) );

            Cl.insert( Cl.delta(*n) );
            Cl.eraseSubsuming( *n );
            Cl.eraseNonMaximal();
        }

        return result;
    }


}
