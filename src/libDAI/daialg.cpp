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


#include "daialg.h"


namespace libDAI {


    Factor calcMarginal( const InferenceAlgorithm & obj, const VarSet & ns, bool reInit ) {
        Factor Pns (ns);
        
        multind mi( ns );

        InferenceAlgorithm *clamped = obj.clone();
        if( !reInit )
            clamped->init();

        std::set<size_t> facs;
        for( size_t I = 0; I < clamped->grm().nrFactors(); I++ )
            if( clamped->grm().factor(I).vars() && ns )
                facs.insert( I );

        Complex logZ0;
        for( size_t j = 0; j < mi.max(); j++ ) {
            // set clamping Factors to delta functions
            std::vector<size_t> vi = mi.vi( j );
            size_t k = 0;
            clamped->grm().backupFactors( facs );
            for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++, k++ )
                clamped->grm().clamp( *n, vi[k] );
            
            // run DAIAlg, calc logZ, store in Pns
            if( clamped->Verbose() >= 2 )
                std::cout << j << ": ";
            if( reInit )
                clamped->init();
            else
                clamped->init(ns);
            clamped->run();

            Complex Z;
            if( j == 0 ) {
                logZ0 = clamped->logZ();
                Z = 1.0;
            } else {
                // subtract logZ0 to avoid very large numbers
                Z = exp(clamped->logZ() - logZ0);
                if( fabs(imag(Z)) > 1e-5 )
                    std::cout << "Marginal:: WARNING: complex Z (" << Z << ")" << std::endl;
            }

            Pns[j] = real(Z);
            
            // restore clamped factors
            clamped->grm().restoreFactors();
        }

        delete clamped;

        return( Pns.normalized() );
    }


    std::vector<Factor> calcPairBeliefs( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit ) {
        // convert ns to vector<VarSet>
        size_t N = ns.size();
        std::vector<Var> vns;
        vns.reserve( N );
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ )
            vns.push_back( *n );

        std::vector<Factor> pairbeliefs;
        pairbeliefs.reserve( N * N );
        for( size_t j = 0; j < N; j++ )
            for( size_t k = 0; k < N; k++ )
                if( j == k )
                    pairbeliefs.push_back(Factor());
                else
                    pairbeliefs.push_back(Factor(vns[j] | vns[k]));

        InferenceAlgorithm *clamped = obj.clone();
        if( !reInit )
            clamped->init();

        std::set<size_t> facs;
        for( size_t I = 0; I < clamped->grm().nrFactors(); I++ )
            if( clamped->grm().factor(I).vars() && ns )
                facs.insert( I );

        Complex logZ0;
        for( size_t j = 0; j < N; j++ ) {
            // clamp Var j to its possible values
            for( size_t j_val = 0; j_val < vns[j].states(); j_val++ ) {
                if( obj.Verbose() >= 2 )
                    std::cout << j << "/" << N-1 << " (" << j_val << "/" << vns[j].states() << "): ";

                clamped->grm().clamp( vns[j], j_val, true );
                if( reInit )
                    clamped->init();
                else
                    clamped->init(ns);
                clamped->run();

                //if( j == 0 )
                //  logZ0 = obj.logZ();
                double Z_xj = 1.0;
                if( j == 0 && j_val == 0 ) {
                    logZ0 = clamped->logZ();
                } else {
                    // subtract logZ0 to avoid very large numbers
                    Complex Z = exp(clamped->logZ() - logZ0);
                    if( fabs(imag(Z)) > 1e-5 )
                        std::cout << "calcPairBelief::  Warning: complex Z: " << Z << std::endl;
                    Z_xj = real(Z);
                }

                for( size_t k = 0; k < N; k++ ) 
                    if( k != j ) {
                        Factor b_k = clamped->belief(vns[k]);
                        for( size_t k_val = 0; k_val < vns[k].states(); k_val++ ) 
                            if( vns[j].label() < vns[k].label() )
                                pairbeliefs[j * N + k][j_val + (k_val * vns[j].states())] = Z_xj * b_k[k_val];
                            else
                                pairbeliefs[j * N + k][k_val + (j_val * vns[k].states())] = Z_xj * b_k[k_val];
                    }

                // restore clamped factors
                clamped->grm().restoreFactors();
            }
        }
        
        delete clamped;

        // Calculate result by taking the geometric average
        std::vector<Factor> result;
        result.reserve( N * (N - 1) / 2 );
        for( size_t j = 0; j < N; j++ )
            for( size_t k = j+1; k < N; k++ )
                result.push_back( (pairbeliefs[j * N + k] * pairbeliefs[k * N + j]) ^ 0.5 );

        return result;
    }


    Factor calcMarginal2ndO( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit ) {
        // returns a a probability distribution whose 1st order interactions
        // are unspecified, whose 2nd order interactions approximate those of 
        // the marginal on ns, and whose higher order interactions are absent.

        std::vector<Factor> pairbeliefs = calcPairBeliefs( obj, ns, reInit );

        Factor Pns (ns);
        for( size_t ij = 0; ij < pairbeliefs.size(); ij++ )
            Pns *= pairbeliefs[ij];
        
        return( Pns.normalized() );
    }


    std::vector<Factor> calcPairBeliefsNew( const InferenceAlgorithm & obj, const VarSet& ns, bool reInit ) {
        std::vector<Factor> result;
        result.reserve( ns.size() * (ns.size() - 1) / 2 );

        InferenceAlgorithm *clamped = obj.clone();
        if( !reInit )
            clamped->init();

        Complex logZ0;
        VarSet::const_iterator nj = ns.begin();
        for( long j = 0; j < (long)ns.size() - 1; j++, nj++ ) {
            size_t k = 0;
            for( VarSet::const_iterator nk = nj; (++nk) != ns.end(); k++ ) {
                Factor pairbelief( *nj | *nk );

            std::set<size_t> facs;
            for( size_t I = 0; I < clamped->grm().nrFactors(); I++ )
                if( clamped->grm().factor(I).vars() && (*nj | *nk) )
                    facs.insert( I );

                // clamp Vars j and k to their possible values
                for( size_t j_val = 0; j_val < nj->states(); j_val++ ) 
                    for( size_t k_val = 0; k_val < nk->states(); k_val++ ) {
                        clamped->grm().backupFactors( facs );
                        clamped->grm().clamp( *nj, j_val );
                        clamped->grm().clamp( *nk, k_val );
                        if( reInit )
                            clamped->init();
                        else
                            clamped->init(ns);
                        clamped->run();

                        double Z_xj = 1.0;
                        if( j_val == 0 && k_val == 0 ) {
                            logZ0 = clamped->logZ();
                        } else {
                            // subtract logZ0 to avoid very large numbers
                            Complex Z = exp(clamped->logZ() - logZ0);
                            if( fabs(imag(Z)) > 1e-5 )
                                std::cout << "calcPairBelief::  Warning: complex Z: " << Z << std::endl;
                            Z_xj = real(Z);
                        }

                        // we assume that j.label() < k.label()
                        // i.e. we make an assumption here about the indexing
                        pairbelief[j_val + (k_val * nj->states())] = Z_xj;

                        // restore clamped factors
                        clamped->grm().restoreFactors();
                    }
            
                result.push_back( pairbelief );
            }
        }
        
        delete clamped;

        assert( result.size() == (ns.size() * (ns.size() - 1) / 2) );

        return result;
    }


}
