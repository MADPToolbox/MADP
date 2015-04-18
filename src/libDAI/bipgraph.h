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


#ifndef __defined_libdai_bipgraph_h
#define __defined_libdai_bipgraph_h


#include <vector>
#include <algorithm>
#include <cassert>


namespace libDAI {


    /// A BipartiteGraph represents a graph with two types of nodes
    ///   (stored as a dense matrix)
    template <class T1, class T2> class BipartiteGraph {
        public:
            /// Type of an edge
            typedef std::pair<size_t,size_t>    edge_type;
            /// Type of neighbors
            typedef std::vector<size_t>         nb_type;
            /// Constant iterator over neighbors
            typedef nb_type::const_iterator     nb_cit;
            

        protected:
            /// Vertices of first type
            std::vector<T1>                     _V1;

            /// Vertices of second type
            std::vector<T2>                     _V2;
            
            /// Edges, which are pairs (v1,v2) with v1 in _V1 and v2 in _V2
            std::vector<edge_type>              _E12;

            /// Conversion matrix that computes which index of _E12 corresponds to a vertex index pair (v1,v2)
            std::vector<std::vector<size_t> >   _E12ind;

            /// Neighbour indices of vertices of first type
            std::vector<nb_type>                _nb1;

            /// Neighbour indices of vertices of second type
            std::vector<nb_type>                _nb2;

            /// Used internally by isTree()
            struct levelType {
                std::vector<size_t> ind1;       // indices of vertices of type 1
                std::vector<size_t> ind2;       // indices of vertices of type 2
            };

        public:
            /// Default constructor
            BipartiteGraph<T1,T2> () {};

            /// Copy constructor
            BipartiteGraph<T1,T2> ( const BipartiteGraph<T1,T2> & x ) : _V1(x._V1), _V2(x._V2), _E12(x._E12), _E12ind(x._E12ind), _nb1(x._nb1), _nb2(x._nb2) {};

            /// Assignment operator
            BipartiteGraph<T1,T2> & operator=(const BipartiteGraph<T1,T2> & x) {
                if( this != &x ) {
                    _V1 =       x._V1;
                    _V2 =       x._V2;
                    _E12 =      x._E12;
                    _E12ind =   x._E12ind;
                    _nb1 =      x._nb1;
                    _nb2 =      x._nb2;
                }
                return *this;
            }
            
            /// Provides read access to node of first type
            const T1 & V1( size_t i1 ) const { return _V1[i1]; }
            /// Provides full access to node of first type
            T1 & V1( size_t i1 ) { return _V1[i1]; }
            /// Provides read access to all nodes of first type
            const std::vector<T1> & V1s() const { return _V1; }
            /// Provides full access to all nodes of first type
            std::vector<T1> & V1s() { return _V1; }
            /// Returns number of nodes of first type
            size_t nr1() const { return _V1.size(); }

            /// Provides read access to node of second type
            const T2 & V2( size_t i2 ) const { return _V2[i2]; }
            /// Provides full access to node of second type
            T2 & V2( size_t i2 ) { return _V2[i2]; }
            /// Provides read access to all nodes of second type
            const std::vector<T2> & V2s() const { return _V2; }
            /// Provides full access to all nodes of second type
            std::vector<T2> & V2s() { return _V2; }
            /// Returns number of nodes of second type
            size_t nr2() const { return _V2.size(); }

            /// Provides read access to edge
            const edge_type & edge(size_t ind) const { return _E12[ind]; }
            /// Provides full access to edge
            edge_type & edge(size_t ind) { return _E12[ind]; }
            /// Provides read access to all edges
            const std::vector<edge_type> & edges() const { return _E12; }
            /// Provides full access to all edges
            std::vector<edge_type> & edges() { return _E12; }
            /// Returns number of edges
            size_t nr_edges() const { return _E12.size(); }

            /// Provides read access to neighbours of node of first type
            const nb_type & nb1( size_t i1 ) const { return _nb1[i1]; }
            /// Provides full access to neighbours of node of first type
            nb_type & nb1( size_t i1 ) { return _nb1[i1]; }

            /// Provides read access to neighbours of node of second type
            const nb_type & nb2( size_t i2 ) const { return _nb2[i2]; }
            /// Provides full access to neighbours of node of second type
            nb_type & nb2( size_t i2 ) { return _nb2[i2]; }

            /// Returns index of a particular node of first type
            size_t find1( const T1 & n1 ) const {
                size_t i1 = find( _V1.begin(), _V1.end(), n1 ) - _V1.begin();
                assert( i1 != _V1.size() );
                return i1;
            }
            /// Returns index of a particular node of second type
            size_t find2( const T2 & n2 ) const {
                size_t i2 = find( _V2.begin(), _V2.end(), n2 ) - _V2.begin();
                assert( i2 != _V2.size() );
                return i2;
            }

            /// Converts the pair of indices (i1,i2) to the corresponding edge index
            size_t VV2E( const size_t i1, const size_t i2 ) const { return _E12ind[i1][i2]; }

            /// Erase node of first type and all edges connecting to it
            void erase1( size_t i1 ) {
                assert( i1 < nr1() );
                _V1.erase( _V1.begin() + i1 );
                for( size_t e = 0; e < _E12.size(); ) {
                    if( _E12[e].first == i1 ) {
                        _E12.erase( _E12.begin() + e );
                    } else {
                        if( _E12[e].first > i1 )
                            _E12[e].first--;
                        e++;
                    }
                }
                Regenerate();
            }

            /// Erase node of second type and all edges connecting to it
            void erase2( size_t i2 ) {
                assert( i2 < nr2() );
                _V2.erase( _V2.begin() + i2 );
                for( size_t e = 0; e < _E12.size(); ) {
                    if( _E12[e].second == i2 ) {
                        _E12.erase( _E12.begin() + e );
                    } else {
                        if( _E12[e].second > i2 )
                            _E12[e].second--;
                        e++;
                    }
                }
                Regenerate();
            }

            /// Regenerates internal structures (_E12ind, _nb1 and _nb2) based upon _V1, _V2 and _E12
            void Regenerate() {
                // Calculate _nb1 and _nb2

                // Start with empty vectors
                _nb1.clear();
                _nb1.resize(_V1.size());
                // Start with empty vectors
                _nb2.clear();
                _nb2.resize(_V2.size());
                // Each edge yields a neighbour pair
                for( std::vector<edge_type>::const_iterator e = _E12.begin(); e != _E12.end(); e++ ) {
                    _nb1[e->first].push_back(e->second);
                    _nb2[e->second].push_back(e->first);
                }
                // Remove duplicates from _nb1
                for( size_t i1 = 0; i1 < _V1.size(); i1++ ) {
                    nb_type::iterator new_end = unique(_nb1[i1].begin(), _nb1[i1].end());
                    _nb1[i1].erase( new_end, _nb1[i1].end() );
                }
                // Remove duplicates from _nb2
                for( size_t i2 = 0; i2 < _V2.size(); i2++ ) {
                    nb_type::iterator new_end = unique(_nb2[i2].begin(), _nb2[i2].end());
                    _nb2[i2].erase( new_end, _nb2[i2].end() );
                }

                // Calculate _E12ind
                
                // Allocate data structures
                _E12ind.clear();
                std::vector<size_t> col(_V2.size(),-1);
                _E12ind.assign(_V1.size(), col);
                // Assign elements
                for( size_t k = 0; k < _E12.size(); k++ )
                    _E12ind[_E12[k].first][_E12[k].second] = k;
            }

            /// Returns true if the graph is connected
            bool isConnected() const {
                if( _V1.size() == 0 )
                    return true;
                else {
                    std::vector<bool> incomponent1( _V1.size(), false );
                    std::vector<bool> incomponent2( _V2.size(), false );

                    incomponent1[0] = true;
                    bool found_new_nodes;
                    do { 
                        found_new_nodes = false;

                        // For all nodes of second type, check if they are connected with the (growing) component
                        for( size_t n2 = 0; n2 < _V2.size(); n2++ )
                            if( !incomponent2[n2] )
                                for( size_t nb_index = 0; nb_index < _nb2[n2].size(); nb_index++ )
                                    if( incomponent1[_nb2[n2][nb_index]] ) {
                                        found_new_nodes = true;
                                        incomponent2[n2] = true;
                                        break;
                                    }

                        // For all nodes of first type, check if they are connected with the (growing) component
                        for( size_t n1 = 0; n1 < _V1.size(); n1++ )
                            if( !incomponent1[n1] )
                                for( size_t nb_index = 0; nb_index < _nb1[n1].size(); nb_index++ )
                                    if( incomponent2[_nb1[n1][nb_index]] ) {
                                        found_new_nodes = true;
                                        incomponent1[n1] = true;
                                        break;
                                    }
                    } while( found_new_nodes );

                    // Check if there are remaining nodes (not in the component)
                    bool all_connected = true;
                    for( size_t n1 = 0; (n1 < _V1.size()) && all_connected; n1++ )
                        if( !incomponent1[n1] )
                            all_connected = false;
                    for( size_t n2 = 0; (n2 < _V2.size()) && all_connected; n2++ )
                        if( !incomponent2[n2] )
                            all_connected = false;

                    return all_connected;
                }
            }

            /// Returns true if the graph is a tree, i.e., singly connected and connected.
            /// This is equivalent to whether for each pair of vertices in the graph, there
            /// exists a unique path in the graph that starts at the first and ends at the
            /// second vertex
            bool isTree() const {
                std::vector<levelType> levels;

                bool result = true;
                size_t nr1 = 0;
                size_t nr2 = 0;

                if( _V1.size() == 0 || _V2.size() == 0 )
                    return true;
                else {
                    levelType newLevel;
                    do {
                        newLevel.ind1.clear();
                        newLevel.ind2.clear();
                        if( levels.size() == 0 ) {
                            size_t n1 = 0;
                            newLevel.ind1 = std::vector<size_t>( 1, n1 ); // add n1 to ind1
                            newLevel.ind2.reserve( _nb1[0].size() ); // add all neighbours of n1 to ind2
                            for( nb_cit i_n2 = _nb1[0].begin(); i_n2 != _nb1[0].end(); i_n2++ )
                                newLevel.ind2.push_back( *i_n2 );
                        } else {
                            const levelType &prevLevel = levels.back();
                            // build newLevel.ind1
                            for( size_t _n2 = 0; _n2 < prevLevel.ind2.size() && result; _n2++ ) { // for all n2 in previous level
                                size_t n2 = prevLevel.ind2[_n2];
                                for( nb_cit i_n1 = _nb2[n2].begin(); i_n1 != _nb2[n2].end() && result; i_n1++ ) { // for all neighbors of n2
                                    if( find( prevLevel.ind1.begin(), prevLevel.ind1.end(), *i_n1 ) == prevLevel.ind1.end() ) { // *i_n1 not in previous level
                                        if( find( newLevel.ind1.begin(), newLevel.ind1.end(), *i_n1 ) != newLevel.ind1.end() )
                                            result = false; // *i_n1 already in new level: we found a cycle
                                        else
                                            newLevel.ind1.push_back( *i_n1 ); // add *i_n1 to new level
                                    }
                                }
                            }
                            // build newLevel.ind2
                            for( size_t _n1 = 0; _n1 < newLevel.ind1.size() && result; _n1++ ) { // for all n1 in this level
                                size_t n1 = newLevel.ind1[_n1];
                                for( nb_cit i_n2 = _nb1[n1].begin(); i_n2 != _nb1[n1].end() && result; i_n2++ ) { // for all neighbors of n1
                                    if( find( prevLevel.ind2.begin(), prevLevel.ind2.end(), *i_n2 ) == prevLevel.ind2.end() ) { // *i_n2 not in previous level
                                        if( find( newLevel.ind2.begin(), newLevel.ind2.end(), *i_n2 ) != newLevel.ind2.end() )
                                            result = false; // *i_n2 already in new level: we found a cycle
                                        else
                                            newLevel.ind2.push_back( *i_n2 ); // add *i_n2 to new level
                                    }
                                }
                            }
                        }
                        levels.push_back( newLevel );
                        nr1 += newLevel.ind1.size();
                        nr2 += newLevel.ind2.size();
                    } while( ((newLevel.ind1.size() != 0) || (newLevel.ind2.size() != 0)) && result );
                    if( nr1 == _V1.size() && nr2 == _V2.size() && result )
                        return true;
                    else 
                        return false;
                }
            }
    
        /// Stream to std::ostream in graphviz .dot syntax
        void display( std::ostream& os ) {
            using namespace std;
            os << "graph G {" << endl;
            os << "node[shape=circle,width=0.4,fixedsize=true];" << endl;
            for( size_t n1 = 0; n1 < _V1.size(); n1++ )
                os << "\tx" << n1 << ";" << endl;
            os << "node[shape=box,width=0.3,height=0.3,fixedsize=true];" << endl;
            for( size_t n2 = 0; n2 < _V2.size(); n2++ )
                os << "\ty" << n2 << ";" << endl;
            for( size_t e = 0; e < _E12.size(); e++ )
                os << "\tx" << _E12[e].first << " -- y" << _E12[e].second << ";" << endl;
            os << "}" << endl;
        }
    };


}


#endif
