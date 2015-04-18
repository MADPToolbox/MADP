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
#include <math.h>
#include "regiongraph.h"
#include "factorgraph.h"
#include "clustergraph.h"


namespace libDAI {


    using namespace std;


    RegionGraph::RegionGraph(const FactorGraph & fg, const std::vector<Region> & ors, const std::vector<Region> & irs, const std::vector<R_edge_t> & edges) : GraphicalModel(), _fg(fg), _rg(), _fac2OR() {
        // Copy outer regions (giving them counting number 1.0)
        ORs().reserve( ors.size() );
        for( vector<Region>::const_iterator alpha = ors.begin(); alpha != ors.end(); alpha++ )
            ORs().push_back( FRegion(Factor(*alpha, 1.0), 1.0) );

        // For each factor, find an outer regions that subsumes that factor.
        // Then, multiply the outer region with that factor.
        for( size_t I = 0; I < _fg.nrFactors(); I++ ) {
            size_t alpha;
            for( alpha = 0; alpha < nr_ORs(); alpha++ )
                if( OR(alpha).vars() >> _fg.factor(I).vars() ) {
                    _fac2OR[I] = alpha;
                    break;
                }
            assert( alpha != nr_ORs() );
        }
        RecomputeORs();
        
        // Copy inner regions
        IRs().reserve( irs.size() );
        for( vector<Region>::const_iterator beta = irs.begin(); beta != irs.end(); beta++ )
            IRs().push_back( *beta );
        
        // Copy edges
        Redges().reserve( edges.size() );
        for( vector<R_edge_t>::const_iterator e = edges.begin(); e != edges.end(); e++ )
            Redges().push_back( *e );
        
        // Regenerate BipartiteGraph internals
        _rg.Regenerate();

        // Check counting numbers
        Check_Counting_Numbers();
    }


    // CVM style
    RegionGraph::RegionGraph(const FactorGraph & fg, const std::vector<VarSet> & cl) : GraphicalModel(), _fg(fg), _rg(), _fac2OR() {
        // Retain only maximal clusters
        ClusterGraph cg( cl );
        cg.eraseNonMaximal();
        
        // Create outer regions, giving them counting number 1.0
        ORs().reserve( cg.size() );
        for( ClusterGraph::const_iterator alpha = cg.begin(); alpha != cg.end(); alpha++ )
            ORs().push_back( FRegion(Factor(*alpha, 1.0), 1.0) );

        // For each factor, find an outer regions that subsumes that factor.
        // Then, multiply the outer region with that factor.
        for( size_t I = 0; I < _fg.nrFactors(); I++ ) {
            size_t alpha;
            for( alpha = 0; alpha < nr_ORs(); alpha++ )
                if( OR(alpha).vars() >> _fg.factor(I).vars() ) {
                    _fac2OR[I] = alpha;
                    break;
                }
            assert( alpha != nr_ORs() );
        }
        RecomputeORs();
        
        // Create inner regions - first pass
        set<VarSet> betas;
        for( ClusterGraph::const_iterator alpha = cg.begin(); alpha != cg.end(); alpha++ )
            for( ClusterGraph::const_iterator alpha2 = alpha; (++alpha2) != cg.end(); ) {
                VarSet intersect = (*alpha) & (*alpha2);
                if( intersect.size() > 0 )
                    betas.insert( intersect );
            }

        // Create inner regions - subsequent passes
        set<VarSet> new_betas;
        do {
            new_betas.clear();
            for( set<VarSet>::const_iterator gamma = betas.begin(); gamma != betas.end(); gamma++ )
                for( set<VarSet>::const_iterator gamma2 = gamma; (++gamma2) != betas.end(); ) {
                    VarSet intersect = (*gamma) & (*gamma2);
                    if( (intersect.size() > 0) && (betas.count(intersect) == 0) )
                        new_betas.insert( intersect );
                }
            betas.insert(new_betas.begin(), new_betas.end());
        } while( new_betas.size() );

        // Create inner regions - store them in the bipartite graph
        IRs().reserve( betas.size() );
        for( set<VarSet>::const_iterator beta = betas.begin(); beta != betas.end(); beta++ )
            IRs().push_back( Region(*beta,NAN) );
        
        // Create edges
        for( size_t beta = 0; beta < nr_IRs(); beta++ ) {
            for( size_t alpha = 0; alpha < nr_ORs(); alpha++ ) {
                if( OR(alpha).vars() >> IR(beta) )
                    Redges().push_back(R_edge_t(alpha,beta));
            }
        }
        
        // Regenerate BipartiteGraph internals
        _rg.Regenerate();

        // Calculate counting numbers
        Calc_Counting_Numbers();

        // Check counting numbers
        Check_Counting_Numbers();
    }


    void RegionGraph::Calc_Counting_Numbers() {
        // Calculates counting numbers of inner regions based upon counting numbers of outer regions
        
        vector<vector<size_t> > ancestors(nr_IRs());
        for( size_t beta = 0; beta < nr_IRs(); beta++ ) {
            IR(beta).c() = NAN;
            for( size_t beta2 = 0; beta2 < nr_IRs(); beta2++ )
                if( (beta2 != beta) && IR(beta2) >> IR(beta) )
                    ancestors[beta].push_back(beta2);
        }

        bool new_counting;
        do {
            new_counting = false;
            for( size_t beta = 0; beta < nr_IRs(); beta++ ) {
                if( std::isnan( IR(beta).c() ) ) {
                    bool has_nan_ancestor = false;
                    for( vector<size_t>::const_iterator beta2 = ancestors[beta].begin(); (beta2 != ancestors[beta].end()) && !has_nan_ancestor; beta2++ )
                        if( std::isnan( IR(*beta2).c() ) )
                            has_nan_ancestor = true;
                    if( !has_nan_ancestor ) {
                        double c = 1.0;
                        for( R_nb_cit alpha = nbIR(beta).begin(); alpha != nbIR(beta).end(); alpha++ )
                            c -= OR(*alpha).c();
                        for( vector<size_t>::const_iterator beta2 = ancestors[beta].begin(); beta2 != ancestors[beta].end(); beta2++ )
                            c -= IR(*beta2).c();
                        IR(beta).c() = c;
                        new_counting = true;
                    }
                }
            }
        } while( new_counting );
    }


    bool RegionGraph::Check_Counting_Numbers() {
        // Checks whether the counting numbers satisfy the fundamental relation
        
        bool all_valid = true;
        for( vector<Var>::const_iterator n = _fg.vars().begin(); n != _fg.vars().end(); n++ ) {
            double c_n = 0.0;
            for( size_t alpha = 0; alpha < nr_ORs(); alpha++ )
                if( OR(alpha).vars() && *n )
                    c_n += OR(alpha).c();
            for( size_t beta = 0; beta < nr_IRs(); beta++ )
                if( IR(beta) && *n )
                    c_n += IR(beta).c();
            if( fabs(c_n - 1.0) > 1e-15 ) {
                all_valid = false;
                cout << "WARNING: counting numbers do not satisfy relation for " << *n << "(c_n = " << c_n << ")." << endl;
            }
        }

        return all_valid;
    }


    void RegionGraph::RecomputeORs() {
        for( size_t alpha = 0; alpha < nr_ORs(); alpha++ )
            OR(alpha).fill( 1.0 );
        for( fac2OR_cit I = _fac2OR.begin(); I != _fac2OR.end(); I++ )
            OR( I->second ) *= _fg.factor( I->first );
    }


    void RegionGraph::RecomputeORs( const VarSet &ns ) {
        for( size_t alpha = 0; alpha < nr_ORs(); alpha++ )
            if( OR(alpha).vars() && ns )
                OR(alpha).fill( 1.0 );
        for( fac2OR_cit I = _fac2OR.begin(); I != _fac2OR.end(); I++ )
            if( OR( I->second ).vars() && ns )
                OR( I->second ) *= _fg.factor( I->first );
    }


    void RegionGraph::RecomputeOR( size_t I ) {
        if( _fac2OR.count(I) ) {
            size_t alpha = _fac2OR[I];
            OR(alpha).fill( 1.0 );
            for( fac2OR_cit I = _fac2OR.begin(); I != _fac2OR.end(); I++ )
                if( I->second == alpha )
                    OR(alpha) *= _fg.factor( I->first );
        }
    }


    ostream & operator << (ostream & os, const RegionGraph & rg) {
        os << "Outer regions" << endl;
        for( size_t alpha = 0; alpha < rg.nr_ORs(); alpha++ )
            os << alpha << ": " << rg.OR(alpha).vars() << ": c = " << rg.OR(alpha).c() << endl;

        os << "Inner regions" << endl;
        for( size_t beta = 0; beta < rg.nr_IRs(); beta++ )
            os << beta << ": " << (VarSet)rg.IR(beta) << ": c = " << rg.IR(beta).c() << endl;

        os << "Edges" << endl;
        for( size_t edge = 0; edge < rg.nr_Redges(); edge++ )
            os << rg.Redge(edge).first << "->" << rg.Redge(edge).second << endl;

        return(os);
    }


}
