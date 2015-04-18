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


#ifndef __defined_libdai_weightedgraph_h
#define __defined_libdai_weightedgraph_h


#include <vector>
#include <map>
#include <iostream>
#include <set>
#include <cassert>
#include <limits.h> // changed from "#include <limits>" to satisfy GCC
                    // 4.3, Matthijs

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>


namespace libDAI {


    /// Directed edge
    class DEdge {
        public:
            size_t  n1, n2;
        
            DEdge() {}
            DEdge( size_t m1, size_t m2 ) : n1(m1), n2(m2) {}
            bool operator==( const DEdge &x ) const {
                return ((n1 == x.n1) && (n2 == x.n2));
            }
            bool operator!=( const DEdge &x ) const {
                return !(*this == x);
            }
            bool operator<( const DEdge &x ) const {
                return( (n1 < x.n1) || ((n1 == x.n1) && (n2 < x.n2)) );
            }
            friend std::ostream & operator << (std::ostream & os, const DEdge & e) {
                os << "(" << e.n1 << "," << e.n2 << ")";
                return os;
            }
    };


    /// Undirected edge
    class UEdge {
        public:
            size_t  n1, n2;
        
            UEdge() {}
            UEdge( size_t m1, size_t m2 ) : n1(m1), n2(m2) {}
            UEdge( const DEdge & e ) : n1(e.n1), n2(e.n2) {}
            bool operator==( const UEdge &x ) {
                return ((n1 == x.n1) && (n2 == x.n2)) || ((n1 == x.n2) && (n2 == x.n1));
            }
            bool operator<( const UEdge &x ) const {
                size_t s = n1, l = n2;
                if( s > l )
                    std::swap( s, l );
                size_t xs = x.n1, xl = x.n2;
                if( xs > xl )
                    std::swap( xs, xl );
                return( (s < xs) || ((s == xs) && (l < xl)) );
            }
            friend std::ostream & operator << (std::ostream & os, const UEdge & e) {
                if( e.n1 < e.n2 )
                    os << "{" << e.n1 << "," << e.n2 << "}";
                else
                    os << "{" << e.n2 << "," << e.n1 << "}";
                return os;
            }
    };


    typedef std::vector<UEdge> UEdgeVec;
    typedef std::vector<DEdge> DEdgeVec;
    std::ostream & operator << (std::ostream & os, const DEdgeVec & rt);
    template<class T> class WeightedGraph : public std::map<UEdge, T> {};
    typedef std::set<UEdge> Graph;


    /// Use Prim's algorithm to construct a minimal spanning tree from the weighted graph Graph
    /// Use implementation in Boost Graph Library
    template<typename T> DEdgeVec MinSpanningTreePrims( const WeightedGraph<T> & Graph ) {
        DEdgeVec result;
        if( Graph.size() > 0 ) {
            using namespace boost;
            using namespace std;
            typedef adjacency_list< vecS, vecS, undirectedS, property<vertex_distance_t, int>, property<edge_weight_t, double> > boostGraph;
            typedef pair<size_t, size_t> E;

            set<size_t> nodes;
            vector<E> edges;
            vector<double> weights;
            edges.reserve( Graph.size() );
            weights.reserve( Graph.size() );
            for( typename WeightedGraph<T>::const_iterator e = Graph.begin(); e != Graph.end(); e++ ) {
                weights.push_back( e->second );
                edges.push_back( E( e->first.n1, e->first.n2 ) );
                nodes.insert( e->first.n1 );
                nodes.insert( e->first.n2 );
            }

            boostGraph g( edges.begin(), edges.end(), weights.begin(), nodes.size() );
            vector< graph_traits< boostGraph >::vertex_descriptor > p( num_vertices(g) );
            prim_minimum_spanning_tree( g, &(p[0]) );

            // Store tree edges in result
            result.reserve( nodes.size() - 1 );
            size_t root = 0;
            for( size_t i = 0; i != p.size(); i++ )
                if( p[i] != i )
                    result.push_back( DEdge( p[i], i ) );
                else
                    root = i;

            // We have to store the minimum spanning tree in the right
            // order, such that for all (i1, j1), (i2, j2) in result,
            // if j1 == i2 then (i1, j1) comes before (i2, j2) in result.
            // We do this by reordering the contents of result, effectively
            // growing the tree starting at the root. At each step, 
            // result[0..N-1] are the edges already added to the tree,
            // whereas the other elements of result still have to be added.
            // The elements of nodes are the vertices that still have to
            // be added to the tree.

            // Start with the root
            nodes.erase( root );
            size_t N = 0;

            // Iteratively add edges and nodes to the growing tree
            while( N != result.size() ) {
                for( size_t e = N; e != result.size(); e++ ) {
                    bool e1_in_tree = !nodes.count( result[e].n1 );
                    if( e1_in_tree ) {
                        nodes.erase( result[e].n2 );
                        swap( result[N], result[e] );
                        N++;
                        break;
                    }
                }
            }
        }

        return result;
    }
            

    /// Use Dijkstra's algorithm to solve the shortest path problem for the weighted graph Graph and source vertex index s
    template<typename T>
    std::map<size_t,size_t> DijkstraShortestPaths( const WeightedGraph<T> & Graph, size_t s ) {
/*
 * from wikipedia 
 *
In the following algorithm, u := Extract_Min(Q) searches for the vertex u in the vertex set Q that has the least d[u] value. That vertex is removed from the set Q and returned to the user.

  function Dijkstra(G, w, s)
     for each vertex v in V[G]                        // Initializations
           d[v] := infinity                                 // Unknown distance function from s to v
           previous[v] := undefined
     d[s] := 0                                        // Distance from s to s
     S := empty set                                   // Set of all visited vertices
     Q := V[G]                                        // Set of all unvisited vertices
     while Q is not an empty set                      // The algorithm itself
           u := Extract_Min(Q)                        // Remove best vertex from priority queue
           S := S union {u}                           // Mark it 'visited'
           for each edge (u,v) outgoing from u
                  if d[u] + w(u,v) < d[v]             // Relax (u,v)
                        d[v] := d[u] + w(u,v) 
                        previous[v] := u

To find the shortest path from s to t:

u := t
while defined previous[u]
      insert u to the beginning of S
      u := previous[u]
	
*/

	// Calculate set of nodes in G
    std::set<size_t> nodes;
	for( typename WeightedGraph<T>::const_iterator e = Graph.begin(); e != Graph.end(); e++ ) {
		nodes.insert( e->first.n1 );
		nodes.insert( e->first.n2 );
	}
    if( !nodes.count( s ) )
        return std::map<size_t, size_t>();

	std::map<size_t, double> d;
	std::map<size_t, size_t> previous;
	for( std::set<size_t>::const_iterator n = nodes.begin(); n != nodes.end(); n++ )
		d[*n] = std::numeric_limits<double>::infinity();

	d[s] = 0;
	std::set<size_t> S;
	std::set<size_t> Q = nodes;

	while( Q.size() ) {
		double least = d[*Q.begin()];
		std::set<size_t>::iterator u_least = Q.begin();
		for( std::set<size_t>::iterator _u = Q.begin(); _u != Q.end(); _u++ )
			if( d[*_u] < least ) {
				u_least = _u;
				least = d[*_u];
			}
		size_t u = *u_least;
		Q.erase( u_least );
			
		S.insert( u );
		for( typename WeightedGraph<T>::const_iterator e = Graph.begin(); e != Graph.end(); e++ ) {
			size_t v;
			if( e->first.n1 == u )
				v = e->first.n2;
			else if( e->first.n2 == u )
				v = e->first.n1;
			else
				continue;
			if( d[u] + e->second < d[v] ) {
				d[v] = d[u] + e->second;
				previous[v] = u;
			}
		}
	}

        return previous;
    }


    /// Calculate rooted tree from a tree T and a root
    DEdgeVec GrowRootedTree( const Graph & T, size_t Root );


    UEdgeVec RandomDRegularGraph( size_t N, size_t d );


}


#endif
