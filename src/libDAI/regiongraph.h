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


#ifndef __defined_libdai_regiongraph_h
#define __defined_libdai_regiongraph_h


#include <iostream>
#include "bipgraph.h"
#include "factorgraph.h"
#include "weightedgraph.h"
#include "graphicalmodel.h"


namespace libDAI {


    /// A Region is a set of variables with a counting number
    class Region : public VarSet {
        protected:
            /// Counting number
            double          _c;

        public:
            /// Default constructor
            Region() : VarSet(), _c(1.0) {}

            /// Construct Region from a VarSet and a counting number
            Region(const VarSet & x, double c) : VarSet(x), _c(c) {}
            
            /// Copy constructor
            Region(const Region & x) : VarSet(x), _c(x._c) {}

            /// Assignment operator
            Region & operator=(const Region & x) {
                if( this != &x ) {
                    VarSet::operator=(x);
                    _c          = x._c;
                }
                return *this;
            }

            /// Provide read access to counting number
            const double & c() const { return _c; }
            /// Provide full access to counting number
            double & c() { return _c; }
    };


    /// A FRegion is a factor with a counting number
    class FRegion : public Factor {
        protected:
            /// Counting number
            double _c;

        public:
            /// Default constructor
            FRegion() : Factor(), _c(1.0) {}

            /// Constructs FRegion from a Factor and a counting number
            FRegion( const Factor & x, double c ) : Factor(x), _c(c) {}
            
            /// Copy constructor
            FRegion( const FRegion & x ) : Factor(x), _c(x._c) {}

            /// Assignment operator
            FRegion & operator=(const FRegion & x) {
                if( this != &x ) {
                    Factor::operator=(x);
                    _c = x._c;
                }
                return *this;
            }

            /// Provide read access to counting number
            const double & c() const { return _c; }
            /// Provide full access to counting number
            double & c() { return _c; }
    };


    /// A RegionGraph is a bipartite graph consisting of outer regions (type FRegion) and inner regions (type Region)
    class RegionGraph : public GraphicalModel {
        private:
            typedef BipartiteGraph<FRegion,Region> BipRegGraph;
            typedef std::map<size_t,size_t>::const_iterator fac2OR_cit;

            FactorGraph _fg;
            BipRegGraph _rg;
            /// Give back the OR index that corresponds to a factor index
            std::map<size_t,size_t> _fac2OR;

        public:
// GraphicalModel interface

            /// Default constructor
            RegionGraph() : GraphicalModel(), _fg(), _rg(), _fac2OR() {}
            
            /// Copy constructor
            RegionGraph( const RegionGraph & x ) : GraphicalModel(x), _fg(x._fg), _rg(x._rg), _fac2OR(x._fac2OR) {}

            /// Assignment operator
            RegionGraph & operator=( const RegionGraph & x ) {
                if( this != &x ) {
                    GraphicalModel::operator=( x );
                    _fg = x._fg;
                    _rg = x._rg;
                    _fac2OR = x._fac2OR;
                }
                return *this;
            }
            
            /// Create (virtual default constructor)
            virtual RegionGraph* create() const {
                return new RegionGraph();
            }

            /// Clone (virtual copy constructor)
            virtual RegionGraph* clone() const {
                return new RegionGraph(*this);
            }

            /// Return connectedness of underlying factor graph
            virtual bool isConnected() const { 
                return _fg.isConnected(); 
            }

            /// Get number of variables
            virtual size_t nrVars() const {
               return _fg.nrVars();
            }

            /// Get const reference to i'th variable
            virtual const Var & var( size_t i ) const {
               return _fg.var(i);
            }

            /// Get const reference to all variables
            virtual const std::vector<Var> & vars() const {
               return _fg.vars();
            }

            /// Get index of variable n
            virtual size_t findVar( const Var & n ) const {
                return _fg.findVar( n );
            }

            /// Get number of factors
            virtual size_t nrFactors() const {
                return _fg.nrFactors();
            }

            /// Get const reference to I'th factor
            virtual const Factor & factor( size_t I ) const {
                return _fg.factor( I );
            }

            /// Get const reference to all factors
            virtual const std::vector<Factor> & factors() const {
                return _fg.factors();
            }

            /// Get index of first factor involving ns
            virtual size_t findFactor( const VarSet &ns ) const {
                return _fg.findFactor( ns );
            }

            /// Return all variables that occur in a factor involving variable n, n itself included
            virtual VarSet Delta( const Var & n ) const {
                return _fg.Delta( n );
            }

            /// Return all variables that occur in a factor involving variable n, n itself excluded
            virtual VarSet delta( const Var & n ) const {
                return _fg.delta( n );
            }

            /// Return all variables that occur in a factor involving some variable in ns, ns itself included
            virtual VarSet Delta( const VarSet & ns ) const {
                return _fg.Delta( ns );
            }

            /// Return all variables that occur in a factor involving some variable in ns, ns itself excluded
            virtual VarSet delta( const VarSet & ns ) const {
                return _fg.delta( ns );
            }



            /// Set the content of the I'th factor and make a backup of its old content if backup == true
            virtual void setFactor( size_t I, const Factor &newFactor, bool backup = false ) {
                _fg.setFactor( I, newFactor, backup ); 
                RecomputeOR( I ); 
            }

            /// Set the contents of all factors as specified by facs and make a backup of the old contents if backup == true
            virtual void setFactors( const std::map<size_t, Factor> & facs, bool backup = false ) {
                _fg.setFactors( facs, backup );
                VarSet ns;
                for( std::map<size_t, Factor>::const_iterator fac = facs.begin(); fac != facs.end(); fac++ )
                    ns |= fac->second.vars();
                RecomputeORs( ns ); 
            }

            /// Clamp variable n to value i (i.e. multiply with a Kronecker delta \f$\delta_{x_n, i}\f$);
            /// If backup == true, make a backup of all factors that are changed
            virtual void clamp( const Var & n, size_t i, bool backup = false ) {
                _fg.clamp( n, i, backup );
                RecomputeORs();
            }

            /// Set all factors interacting with variable n to 1
            virtual void makeCavity( const Var & n ) {
                _fg.makeCavity( n );
                RecomputeORs();
            }

            /// Backup the factors specified by indices in facs
            virtual void backupFactors( const std::set<size_t> & facs ) {
                _fg.backupFactors( facs );
            }

            /// Restore all factors to the backup copies
            virtual void restoreFactors() {
                _fg.restoreFactors();
                RecomputeORs();
            }


// RegionGraph specific methods

        public:
            typedef BipRegGraph::nb_type           R_nb_t;
            typedef R_nb_t::const_iterator         R_nb_cit;
            typedef BipRegGraph::edge_type         R_edge_t;

            /// Constructs a RegionGraph from a FactorGraph
            RegionGraph(const FactorGraph & fg) : GraphicalModel(), _fg(fg), _rg(), _fac2OR() {}

            /// Constructs a RegionGraph from a FactorGraph, a vector of outer regions, a vector of inner regions and a vector of edges
            RegionGraph(const FactorGraph & fg, const std::vector<Region> & ors, const std::vector<Region> & irs, const std::vector<R_edge_t> & edges);
            
            /// Constructs a RegionGraph from a FactorGraph and a vector of outer VarSets (CVM style)
            RegionGraph(const FactorGraph & fg, const std::vector<VarSet> & cl);


            /// Provides read access to outer region
            const FRegion & OR(long alpha) const { return _rg.V1(alpha); }
            /// Provides access to outer region
            FRegion & OR(long alpha) { return _rg.V1(alpha); }
            /// Provides read access to all outer regions
            const std::vector<FRegion> & ORs() const { return _rg.V1s(); }
            /// Provides access to all outer regions
            std::vector<FRegion> &ORs() { return _rg.V1s(); }
            /// Returns number of outer regions
            size_t nr_ORs() const { return _rg.V1s().size(); }

            /// Provides read access to inner region
            const Region & IR(long beta) const { return _rg.V2(beta); }
            /// Provides access to inner region
            Region & IR(long beta) { return _rg.V2(beta); }
            /// Provides read access to all inner regions
            const std::vector<Region> & IRs() const { return _rg.V2s(); }
            /// Provides access to all inner regions
            std::vector<Region> & IRs() { return _rg.V2s(); }
            /// Returns number of inner regions
            size_t nr_IRs() const { return _rg.V2s().size(); }

            /// Provides read access to edge
            const R_edge_t & Redge(size_t ind) const { return _rg.edge(ind); }
            /// Provides full access to edge
            R_edge_t & Redge(size_t ind) { return _rg.edge(ind); }
            /// Provides read access to all edges
            const std::vector<R_edge_t> & Redges() const { return _rg.edges(); }
            /// Provides full access to all edges
            std::vector<R_edge_t> & Redges() { return _rg.edges(); }
            /// Returns number of edges
            size_t nr_Redges() const { return _rg.edges().size(); }

            /// Provides read access to neighbours of outer region
            const R_nb_t & nbOR( size_t i1 ) const { return _rg.nb1(i1); }
            /// Provides full access to neighbours of outer region
            R_nb_t & nbOR( size_t i1 ) { return _rg.nb1(i1); }

            /// Provides read access to neighbours of inner region
            const R_nb_t & nbIR( size_t i2 ) const { return _rg.nb2(i2); }
            /// Provides full access to neighbours of inner region
            R_nb_t & nbIR( size_t i2 ) { return _rg.nb2(i2); }

            /// Converts the pair of outer/inner region indices (i1,i2) to the corresponding edge index
            size_t ORIR2E( const size_t i1, const size_t i2 ) const { return _rg.VV2E(i1, i2); }

            /// Regenerates BipRegGraph internals
            void Regenerate() { _rg.Regenerate(); }
            
            /// Gives the outer region in which a factor lives
            size_t getFac2OR( size_t I ) const { 
                std::map<size_t,size_t>::const_iterator x = _fac2OR.find(I);
                assert( x != _fac2OR.end() );
                return x->second;
            }

            /// Sets the outer region in which a factor lives
            void setFac2OR( size_t I, size_t alpha ) { _fac2OR[I] = alpha; }

            /// Counts the number of ORs in which a factor lives
            size_t nrFac2OR( size_t I ) const { return _fac2OR.count(I); }
            
            /// Calculates counting numbers of inner regions based upon counting numbers of outer regions
            void Calc_Counting_Numbers();
            /// Check whether the counting numbers are valid
            bool Check_Counting_Numbers();

            /// Recompute all outer regions
            void RecomputeORs();

            /// Recompute all outer regions involving the variables in ns
            void RecomputeORs( const VarSet & ns );

            /// Recompute all outer regions involving factor I
            void RecomputeOR( size_t I );

            /// Send RegionGraph to output stream
            friend std::ostream & operator << ( std::ostream & os, const RegionGraph & rg );
    };


}


#endif
