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


#ifndef __defined_libdai_clustergraph_h
#define __defined_libdai_clustergraph_h


#include <set>
#include <vector>
#include "varset.h"
#include "bipgraph.h"


namespace libDAI {


    /// A ClusterGraph is a hypergraph with VarSets as nodes.
    /// It is implemented as bipartite graph with variable nodes
    /// and cluster (VarSet) nodes. The additional
    /// functionality compared to a simple set<VarSet> is
    /// finding maximal clusters, finding cliques, etc...
    class ClusterGraph : public BipartiteGraph<Var,VarSet> {
        private:
            typedef BipartiteGraph<Var,VarSet> BC;  // BC stands for BaseClass

        public:
            /// Default constructor
            ClusterGraph() : BC() {}

            /// Construct from vector<VarSet>
            ClusterGraph( const std::vector<VarSet> & cls );
            
            /// Copy constructor
            ClusterGraph( const ClusterGraph &x ) : BC(x) {}

            /// Assignment operator
            ClusterGraph& operator=( const ClusterGraph &x ) {
                if( this != &x ) {
                    BC::operator=(x);
                }
                return *this;
            }

            /// Iterators
            typedef std::vector<VarSet>::iterator iterator;
            typedef std::vector<VarSet>::const_iterator const_iterator;
            const_iterator begin() { return _V2.begin(); }
            const_iterator end() { return _V2.end(); }

            /// Returns true if V2s(I) is a maximal member of *this under inclusion
            bool isMaximal( size_t I ) const {
                assert( I < nr2() );
                const VarSet & clI = V2(I);
                bool maximal = true;
                // The following may not be optimal, since it may repeatedly test the same cluster *J
                for( nb_cit i = nb2(I).begin(); i != nb2(I).end() && maximal; i++ )
                    for( nb_cit J = nb1(*i).begin(); J != nb1(*i).end() && maximal; J++ ) 
                        if( (*J != I) && (clI << V2(*J)) )
                            maximal = false;
                return maximal;
            }

            /// Erase all VarSets that are not maximal
            ClusterGraph& eraseNonMaximal() {
                for( size_t I = 0; I < nr2(); ) {
                    if( !isMaximal(I) )
                        erase2(I);
                    else
                        I++;
                }
                return *this;
            }

            /// Return union of all members
            VarSet vars() const {
                VarSet result( V1s() );
                return result;
            }

            /// Return number of clusters
            size_t size() const {
                return nr2();
            }

            /// Returns true if vars with indices i1 and i2 are adjacent, i.e., both contained in the same cluster
            bool adj( size_t i1, size_t i2 ) {
                bool result = false;
                for( nb_cit J = nb1(i1).begin(); J != nb1(i1).end() && !result; J++ ) 
                    if( find( nb2(*J).begin(), nb2(*J).end(), i2 ) != nb2(*J).end() )
                        result = true;
                return result;
            }
            
            /// Returns union of clusters that contain the variable with index i
            VarSet Delta( size_t i ) const {
                VarSet result;
                for( nb_cit J = nb1(i).begin(); J != nb1(i).end(); J++ )
                    result |= V2(*J);
                return result;
            }

            /// Returns union of clusters that contain n
            VarSet Delta( const Var& n ) const {
                size_t i = find1( n );
                return Delta( i );
            }

            /// Returns true if n1 and n2 are adjacent, i.e., both contained in the same cluster
            bool adj( const Var& n1, const Var& n2 ) {
                size_t i1 = find1( n1 );
                size_t i2 = find1( n2 );
                return adj( i1, i2 );
            }

            /// Inserts a cluster (if it does not already exist)
            void insert( const VarSet &cl ) {
                if( find( V2s().begin(), V2s().end(), cl ) == V2s().end() ) {
                    size_t i2 = nr2();
                    V2s().push_back( cl );
                    // add variables (if necessary) and edges
                    for( VarSet::const_iterator n = cl.begin(); n != cl.end(); n++ ) {
                        std::vector<Var>::iterator it = find( V1s().begin(), V1s().end(), *n );
                        if( it == V1s().end() ) {
                            size_t i1 = nr1();
                            V1s().push_back( *n );
                            edges().push_back( edge_type(i1, i2) );
                        } else
                            edges().push_back( edge_type(it - V1s().begin(), i2) );
                    }
                    Regenerate();
                }
            }

            /// Returns union of clusters that contain n, minus n
            VarSet delta( const Var& n ) const {
                return Delta( n ) / n;
            }

            /// Returns union of clusters that contain variable with index i, minus this variable
            VarSet delta( size_t i ) const {
                return Delta( i ) / V1(i);
            }
            
            /// Returns indices of variables in Delta(n) where n is the variable with index i
            std::vector<size_t> ind_Delta( size_t i ) const {
                std::vector<size_t> result;
                for( nb_cit J = nb1(i).begin(); J != nb1(i).end(); J++ )
                    result.insert( result.end(), nb2(*J).begin(), nb2(*J).end() );
                std::vector<size_t>::iterator it = unique( result.begin(), result.end() );
                result.erase( it, result.end() );
                return result;
            }

            /// Returns indices of variables in Delta(n)
            std::vector<size_t> ind_Delta( const Var& n ) const {
                return ind_Delta( find1( n ) );
            }

            /// Returns indices of variables in delta(n) where n is the variable with index i
            std::vector<size_t> ind_delta( size_t i ) const {
                std::vector<size_t> result = ind_Delta( V1(i) );
                std::vector<size_t>::iterator it = find( result.begin(), result.end(), i );
                result.erase( it );
                return result;
            }

            /// Returns indices of variables in delta(n)
            std::vector<size_t> ind_delta( const Var& n ) const {
                return ind_delta( find1( n ) );
            }

            /// Erases all members that contain n where n is the variable with index i
            ClusterGraph& eraseSubsuming( size_t i ) {
                while( nb1(i).size() ) {
                    erase2( nb1(i)[0] );
                }
                return *this;
            }
            
            /// Erases all members that contain n
            ClusterGraph& eraseSubsuming( const Var& n ) {
                return eraseSubsuming( find1( n ) );
            }

            /// Send to output stream
            friend std::ostream & operator << ( std::ostream & os, const ClusterGraph & cl ) {
                //os << "<" << cl.nr1() << " vars, " << cl.nr2() << " clusters, " << cl.nr_edges() << " edges>: {";
                os << "{";
                size_t I = 0;
                if( I != cl.nr2() )
                    os << cl.V2(I);
                for( ; I != cl.nr2(); I++ )
                    os << ", " << cl.V2(I);
                os << "}";
                return os;
            }

            /// Convert to vector<VarSet>
            std::vector<VarSet> toVector() const {
                std::vector<VarSet> result = V2s();
                return result;
            }

            /// Calculate cost of eliminating the variable with index i,
            /// using as a measure "number of added edges in the adjacency graph"
            /// where the adjacency graph has the variables as its nodes and
            /// connects nodes i1 and i2 iff i1 and i2 occur in some common cluster
            size_t eliminationCost( size_t i ) {
                std::vector<size_t> id_n = ind_delta( i );

                size_t cost = 0;

                // for each unordered pair {i1,i2} adjacent to n
                for( size_t _i1 = 0; _i1 < id_n.size(); _i1++ )
                    for( size_t _i2 = _i1 + 1; _i2 < id_n.size(); _i2++ ) {
                        // if i1 and i2 are not adjacent, eliminating n would make them adjacent
                        if( !adj(id_n[_i1], id_n[_i2]) )
                            cost++;
                    }

                return cost;
            }

            /// Calculate cost of eliminating variable n
            /// using as a measure "number of added edges in the adjacency graph"
            /// where the adjacency graph has the variables as its nodes and
            /// connects nodes i1 and i2 iff i1 and i2 occur in some common cluster
            size_t eliminationCost( const Var& n ) {
                return eliminationCost( find1( n ) );
            }

            /// Perform Variable Elimination without Probs, i.e. only keeping track of
            /// the interactions that are created along the way.
            /// Input:  a set of outer clusters and an elimination sequence
            /// Output: a set of elimination "cliques"
            ClusterGraph VarElim( const std::vector<Var> &ElimSeq ) const;

            /// As Taylan does it
            ClusterGraph VarElim_MinFill() const;

            /// As Taylan does it, but return the elimination sequence instead of resulting ClusterGraph
            std::vector<Var> MinFill() const;
    };


}


#endif
