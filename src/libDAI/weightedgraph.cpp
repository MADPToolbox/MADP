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


#include <algorithm>
#include <cassert>
#include "weightedgraph.h"
#include "util.h"


namespace libDAI {


    using namespace std;


    std::ostream & operator << (std::ostream & os, const DEdgeVec & rt) {
        os << "[";
        for( size_t n = 0; n < rt.size(); n++ )
            os << rt[n] << ", ";
        os << "]";
        return os;
    }


    DEdgeVec GrowRootedTree( const Graph & T, size_t Root ) {
        DEdgeVec result;
        if( T.size() == 0 )
            return result;
        else {
            // Make a copy
            Graph Gr = T;

            // Nodes in the tree
            set<size_t> treeV;

            // Start with the root
            treeV.insert( Root );
            
            // Keep adding edges until done
            while( !(Gr.empty()) )
                for( Graph::iterator e = Gr.begin(); e != Gr.end(); ) {
                    bool e1_in_treeV = treeV.count( e->n1 );
                    bool e2_in_treeV = treeV.count( e->n2 );
                    assert( !(e1_in_treeV && e2_in_treeV) );
                    if( e1_in_treeV ) {
                        // Add directed edge, pointing away from the root
                        result.push_back( DEdge( e->n1, e->n2 ) );
                        treeV.insert( e->n2 );
                        // Erase the edge
                        Gr.erase( e++ );
                    } else if( e2_in_treeV ) {
                        result.push_back( DEdge( e->n2, e->n1 ) );
                        treeV.insert( e->n1 );
                        // Erase the edge
                        Gr.erase( e++ );
                    } else
                        e++;
                }

            return result;
        }
    }


    UEdgeVec RandomDRegularGraph( size_t N, size_t d ) {
        // Algorithm 1 in "Generating random regular graphs quickly"
        // by A. Steger and N.C. Wormald
        //
        // Draws a random graph with size N and uniform degree d
        // from an almost uniform probability distribution over these graphs
        // (which becomes uniform in the limit that d is small and N goes
        // to infinity).
        
        assert( (N * d) % 2 == 0 );

        bool ready = false;
        UEdgeVec G;

        size_t tries = 0;
        while( !ready ) {
            tries++;

            // Start with N*d points {0,1,...,N*d-1} (N*d even) in N groups.
            // Put U = {0,1,...,N*d-1}. (U denotes the set of unpaired points.)
            vector<size_t> U;
            U.reserve( N * d );
            for( size_t i = 0; i < N * d; i++ )
                U.push_back( i );

            // Repeat the following until no suitable pair can be found: Choose
            // two random points i and j in U, and if they are suitable, pair
            // i with j and delete i and j from U.
            G.clear();
            bool finished = false;
            while( !finished ) {
                random_shuffle( U.begin(), U.end() );
                size_t i1, i2;
                bool suit_pair_found = false;
                for( i1 = 0; i1 < U.size()-1 && !suit_pair_found; i1++ )
                    for( i2 = i1+1; i2 < U.size() && !suit_pair_found; i2++ )
                        if( (U[i1] / d) != (U[i2] / d) ) {
                            // they are suitable
                            suit_pair_found = true;
                            G.push_back( UEdge( U[i1] / d, U[i2] / d ) );
                            U.erase( U.begin() + i2 );  // first remove largest
                            U.erase( U.begin() + i1 );  // remove smallest
                        }
                if( !suit_pair_found || U.empty() )
                    finished = true;
            }

            if( U.empty() ) {
                // G is a graph with edge from vertex r to vertex s if and only if
                // there is a pair containing points in the r'th and s'th groups.
                // If G is d-regular, output, otherwise return to Step 1.

                vector<size_t> degrees;
                degrees.resize( N, 0 );
                for( UEdgeVec::const_iterator e = G.begin(); e != G.end(); e++ ) {
                    degrees[e->n1]++;
                    degrees[e->n2]++;
                }
                ready = true;
                for( size_t n = 0; n < N; n++ )
                    if( degrees[n] != d ) {
                        ready = false;
                        break;
                    }
            } else
                ready = false;
        }

        return G;
    }


}
