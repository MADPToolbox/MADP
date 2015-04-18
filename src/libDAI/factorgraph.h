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


#ifndef __defined_libdai_factorgraph_h
#define __defined_libdai_factorgraph_h


#include <iostream>
#include <map>
#include "bipgraph.h"
#include "factor.h"
#include "graphicalmodel.h"


namespace libDAI {


    class FactorGraph : public GraphicalModel {
        private:
            typedef BipartiteGraph<Var,Factor> BipFacGraph;

            BipFacGraph                        _fg;
            std::map<size_t,Factor>            _backupFactors;


        public:
// GraphicalModel interface

            /// Default constructor
            FactorGraph() : GraphicalModel(), _fg(), _backupFactors() {}
            
            /// Copy constructor
            FactorGraph( const FactorGraph & x ) : GraphicalModel(x), _fg(x._fg), _backupFactors(x._backupFactors) {}

            /// Assignment operator
            FactorGraph & operator=( const FactorGraph & x ) {
                if( this != &x ) {
                    GraphicalModel::operator=( x );
                    _fg = x._fg;
                    _backupFactors = x._backupFactors;
                }
                return *this;
            }

            /// Create (virtual default constructor)
            virtual FactorGraph* create() const {
                return new FactorGraph(*this);
            }

            /// Clone (virtual copy constructor)
            virtual FactorGraph* clone() const {
                return new FactorGraph();
            }


            /// Get number of variables
            virtual size_t nrVars() const { 
                return _fg.V1s().size(); 
            }

            /// Get const reference to i'th variable
            virtual const Var & var( size_t i ) const { 
                return _fg.V1(i); 
            }

            /// Get const reference to all variables
            virtual const std::vector<Var> & vars() const { 
                return _fg.V1s(); 
            }

            /// Get index of variable n
            virtual size_t findVar( const Var & n ) const {
                size_t i = find( vars().begin(), vars().end(), n ) - vars().begin();
                assert( i != nrVars() );
                return i;
            }

            /// Get set of indexes for set of variables
            virtual std::set<size_t> findVars( VarSet &ns ) const {
                std::set<size_t> indexes;
                for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ )
                    indexes.insert( findVar( *n ) );
                return indexes;
            }

            /// Get number of factors
            virtual size_t nrFactors() const { 
                return _fg.V2s().size(); 
            }

            /// Get const reference to I'th factor
            virtual const Factor & factor( size_t I ) const { 
                return _fg.V2(I); 
            }

            /// Get const reference to all factors
            virtual const std::vector<Factor> & factors() const { 
                return _fg.V2s(); 
            }

            /// Get index of first factor involving ns
            virtual size_t findFactor( const VarSet &ns ) const {
                size_t I;
                for( I = 0; I < nrFactors(); I++ )
                    if( factor(I).vars() == ns )
                        break;
                assert( I != nrFactors() );
                return I;
            }

            /// Return all variables that occur in a factor involving variable n, n itself included
            virtual VarSet Delta( const Var & n ) const;

            /// Return all variables that occur in a factor involving some variable in ns, ns itself included
            virtual VarSet Delta( const VarSet &ns ) const;

            /// Return all variables that occur in a factor involving variable n, n itself excluded
            virtual VarSet delta( const Var & n ) const;

            /// Return all variables that occur in a factor involving some variable in ns, ns itself excluded
            virtual VarSet delta( const VarSet & ns ) const {
                return Delta( ns ) / ns;
            }

            /// Set the content of the I'th factor and make a backup of its old content if backup == true
            virtual void setFactor( size_t I, const Factor &newFactor, bool backup = false ) {
                assert( newFactor.vars() == _fg.V2(I).vars() ); 
                if( backup )
                    backupFactor( I );
                _fg.V2(I) = newFactor; 
            }

            /// Set the contents of all factors as specified by facs and make a backup of the old contents if backup == true
            virtual void setFactors( const std::map<size_t, Factor> & facs, bool backup = false ) {
                for( std::map<size_t, Factor>::const_iterator fac = facs.begin(); fac != facs.end(); fac++ ) {
                    if( backup )
                        backupFactor( fac->first );
                    setFactor( fac->first, fac->second );
                }
            }

            /// Clamp variable n to value i (i.e. multiply with a Kronecker delta \f$\delta_{x_n, i}\f$);
            /// If backup == true, make a backup of all factors that are changed
            virtual void clamp( const Var & n, size_t i, bool backup = false );

            /// Set all factors interacting with variable n to 1
            virtual void makeCavity( const Var & n );

            /// Backup the factors specified by indices in facs
            virtual void backupFactors( const std::set<size_t> & facs );

            /// Restore all factors to the backup copies
            virtual void restoreFactors();


// FactorGraph specific methods

            FactorGraph( const std::vector<Factor> &P );

            typedef BipFacGraph::nb_cit    nb_cit;
            typedef BipFacGraph::nb_type   nb_type;
            typedef BipFacGraph::edge_type edge_type;

            const edge_type & edge(size_t ind) const { return _fg.edge(ind); }
            const std::vector<BipFacGraph::edge_type> & edges() const { return _fg.edges(); }
            size_t nrEdges() const { return _fg.nr_edges(); }
            size_t edge( size_t i, size_t I ) const { return _fg.VV2E(i,I); }

            void Regenerate() { _fg.Regenerate(); }
            bool isConnected() const { return _fg.isConnected(); }
            bool isTree() const { return _fg.isTree(); }

            /**\brief FRANS: provides acces to neigbors of the i1-th variable.
             *
             * the 1 in 'i1' indicates that it is a index of type 1 
             *      (i.e. a index of a variable)
             *
             * _fg.nb1(i1) returns the set of ('i2') indices of the neighbors
             * of i1.
             *
             * -these neigbors are factors.
             * -the set has the data type 'nb_type' (which is a vector<size_t>).
             *
             */             
            const nb_type & nbV( size_t i1 ) const { return _fg.nb1(i1); }

            /**\brief FRANS: provides acces to neigbors of the i2-th factor.
             *
             * the 2 in 'i2' indicates that it is a index of type 2 
             *      (i.e. a index of a factor)
             *
             * _fg.nb2(i2) returns the set of ('i1') indices of the neighbors
             * of i2.
             *
             * -these neighbors are variables
             * -the set has the data type 'nb_type' (which is a vector<size_t>).
             *
             */             
            const nb_type & nbF( size_t i2 ) const { return _fg.nb2(i2); }

            friend std::ostream& operator << (std::ostream& os, const FactorGraph& fg);
            friend std::istream& operator >> (std::istream& is, FactorGraph& fg);

            void ReadFromFile(const char *filename);
            void WriteToFile(const char *filename) const;

            void display( std::ostream& os ) const;

            std::vector<VarSet> Cliques() const;

            // Clamp variable v_i to value state (i.e. multiply with a Kronecker delta \f$\delta_{x_{v_i},x}\f$);
            // This version changes the factor graph structure and thus returns a newly constructed FactorGraph
            // and keeps the current one constant, contrary to clamp()
            FactorGraph clamped( const Var & v_i, size_t x ) const;

            bool isPairwise() const;
            bool isBinary() const;

        private:
            void restoreFactors( const VarSet &ns );
            void backupFactors( const VarSet &ns );
            void restoreFactor( size_t I );
            void backupFactor( size_t I );
    };


    bool hasShortLoops(const std::vector<Factor> &P);
    bool hasNegatives(const std::vector<Factor> &P);
    void RemoveShortLoops(std::vector<Factor> &P);



}


#endif
