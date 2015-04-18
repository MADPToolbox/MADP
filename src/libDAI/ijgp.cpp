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


#include <iostream>
#include <algorithm>
#include "ijgp.h"
#include "clustergraph.h"
#include "exceptions.h"
#include "diffs.h"


namespace libDAI {


    using namespace std;


    const char *IJGP::Name = "IJGP";


    bool IJGP::initProps() {
        if( !HasProperty("updates") )
            return false;
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("verbose") )
            return false;

        Props.updates = FromStringTo<UpdateType>("updates");
        Props.tol     = FromStringTo<double>("tol");
        Props.maxiter = FromStringTo<size_t>("maxiter");
        Props.verbose = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            Props.damping = FromStringTo<double>("damping");
        else
            Props.damping = 0.0;
        if( HasProperty("i") )
            Props.i = FromStringTo<size_t>("i");
        else
            Props.i = 0xFFFFFFFFUL;

        return true;
    }


    IJGP::IJGP( const FactorGraph &fg, const Properties &opts ) : DAIAlgRG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _Qa(), _Qb(), _mes() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        ClusterGraph _cg;

        // Copy factor varsets
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            _cg.insert( grm().factor(I).vars() );
        // Retain only maximal clusters
        _cg.eraseNonMaximal();
        // Obtain (heuristically) optimal elimination sequence
        vector<Var> ElimOrder = _cg.MinFill();
        vector<size_t> ElimSeq;
        ElimSeq.reserve( ElimOrder.size() );
        for( size_t i = 0; i < ElimOrder.size(); i++ )
            ElimSeq.push_back( grm().findVar(ElimOrder[i]) );
        if( Verbose() >= 3 ) {
            cout << "MinFill variable elimination order:" << endl;
            for( size_t i = 0; i < ElimOrder.size(); i++ ) {
                if( i != 0 )
                    cout << ", ";
                cout << ElimOrder[i];
            }
            cout << endl;
        }

        // Each variable has an associated bucket
        vector<Bucket> buckets;
        buckets.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            buckets.push_back( Bucket(i) );

        // Each minibucket will correspond to an outer region
        vector<Bucket> minibuckets;
        minibuckets.reserve( grm().nrVars() );

        // Place each factor in the bucket of the lowest (first-to-eliminate) index variable in its domain
        for( size_t I = 0; I < grm().nrFactors(); I++ ) {
            VarSet Ivars = grm().factor(I).vars();
            size_t lowest = findElimOrderIndex( ElimOrder, Ivars );
            if( Verbose() >= 3 )
                cout << Ivars << "->" << ElimOrder[lowest] << " " << endl;

            buckets[ElimSeq[lowest]].push_back( BucketEntry( Ivars, true, I ) );
        }

        // For each bucket
        size_t nrArcs = 0;
        for( size_t _i = 0; _i < ElimOrder.size(); _i++ ) {
            Var vi = ElimOrder[_i];
            size_t i = ElimSeq[_i];

            // Partition bucket into mini-buckets
            vector<Bucket> mb_i = partitionBucket( buckets[i], Props.i );
            if( Verbose() >= 3 )
                cout << "number of minibuckets created: " << mb_i.size() << endl;
            nrArcs += mb_i.size() - 1;
            for( size_t mb_i_index = 0; mb_i_index < mb_i.size(); mb_i_index++ ) {
                minibuckets.push_back( mb_i[mb_i_index] );

                // Eliminate this variable
                VarSet vars = minibuckets.back().vars();
                if( Verbose() >= 3 )
                    cout << "  mb.vars = " << vars << endl;
                vars /= vi;
                if( vars.size() ) {
                    // Find lowest index variable in vars
                    size_t _j = findElimOrderIndex( ElimOrder, vars, _i+1 );
                    // j is the index of that variable
                    size_t j = ElimSeq[_j];
                    // Insert message into the corresponding bucket of j
                    buckets[j].push_back( BucketEntry(vars, false, minibuckets.size()-1 ) );
                    nrArcs++;
                    if( Verbose() >= 3 )
                        cout << "  added arc to bucket" << grm().var(j) << ", i.e. (" << _i << "," << _j << ")" << endl;
                }
            }
        }

        // Each minibucket becomes an outer region that contains the factors in that minibucket
        // For each message in the minibucket, we add an inner region (corresponding to an
        // arc of the join graph) and two edges
        // We also connect minibuckets of the same var with an inner region
        grm().ORs().reserve( minibuckets.size() );
        grm().IRs().reserve( nrArcs );
        grm().Redges().reserve( 2 * nrArcs );
        for( size_t alpha = 0; alpha < minibuckets.size(); alpha++ ) {
            Bucket & mb = minibuckets[alpha];
            grm().ORs().push_back( FRegion( Factor(mb.vars(), 1.0), 1.0 ) );
            for( size_t bE = 0; bE < mb.size(); bE++ )
                if( mb[bE].isFactor )
                    grm().setFac2OR( mb[bE].pointsTo, alpha );
                else {
                    size_t alpha2 = mb[bE].pointsTo;
                    grm().Redges().push_back( RegionGraph::R_edge_t( alpha, grm().IRs().size() ) );
                    grm().Redges().push_back( RegionGraph::R_edge_t( alpha2, grm().IRs().size() ) );
                    grm().IRs().push_back( Region( grm().OR(alpha).vars() & grm().OR(alpha2).vars(), -1.0 ) );
                }
            if( alpha )
                if( minibuckets[alpha-1].var == mb.var ) {
                    size_t alpha2 = alpha - 1;
                    grm().Redges().push_back( RegionGraph::R_edge_t( alpha, grm().IRs().size() ) );
                    grm().Redges().push_back( RegionGraph::R_edge_t( alpha2, grm().IRs().size() ) );
                    grm().IRs().push_back( Region( grm().var(mb.var), -1.0 ) );
                }
        }
        grm().Regenerate();
        grm().RecomputeORs();

        // Create messages and beliefs
        _Qa.clear();
        _Qa.reserve( grm().nr_ORs() );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa.push_back( grm().OR(alpha) );

        _Qb.clear();
        _Qb.reserve( grm().nr_IRs() );
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) 
            _Qb.push_back( Factor( grm().IR(beta), 1.0 ) );

        _mes.clear();
        _mes.reserve( grm().nr_Redges() );
        for( size_t e = 0; e < grm().nr_Redges(); e++ )
            _mes.push_back( Factor( grm().IR(grm().Redge(e).second), 1.0 ) );

        // Check counting numbers
        grm().Check_Counting_Numbers();

        if( Verbose() >= 3 )
            cout << "Resulting regiongraph: " << grm() << endl;
    }


    size_t IJGP::findElimOrderIndex( const std::vector<Var> & ElimOrder, const VarSet & vars, size_t start ) {
        size_t lowest = start;
        for( ; lowest < ElimOrder.size(); lowest++ )
            if( vars && ElimOrder[lowest] )
                break;
        assert( lowest < ElimOrder.size() );
        return lowest;
    }


    // partition a bucket into mini-buckets of i variables at most
    vector<IJGP::Bucket> IJGP::partitionBucket( const Bucket & b, size_t i ) {
        typedef vector<Bucket> vecBucket;
        vecBucket minibuckets;

        // Create canonical partitioning
        // in which subsumed functions are combined into mini-buckets
        // For each function in the bucket, we need to either find an
        // existing mini-bucket to which it can be assigned or we need
        // to create a new mini-bucket
        for( size_t bE = 0; bE < b.size(); bE++ ) {
            size_t mb = 0;

            // Find a mini-bucket that is larger than the current function
            for( mb = 0; mb < minibuckets.size(); mb++ )
                if( b[bE] << minibuckets[mb].vars() )
                    break;
            // If found, add the current function to that mini-bucket
            // and go to the next function
            if( mb < minibuckets.size() ) {
                minibuckets[mb].push_back( b[bE] );
                continue;
            }

            // Find a mini-bucket that is subsumed by the current function
            for( mb = 0; mb < minibuckets.size(); mb++ )
                if( b[bE] >> minibuckets[mb].vars() )
                    break;
            // If found, add the current function to that mini-bucket
            // and go to the next function
            if( mb < minibuckets.size() ) {
                minibuckets[mb].push_back( b[bE] );
                continue;
            }

            // We have not found a corresponding mini-bucket and need to
            // create a new one
            minibuckets.push_back( Bucket(b.var) );
            minibuckets.back().push_back( b[bE] );
        }

        if( Verbose() >= 3 ) {
            cout << "Canonical partitioning of " << b << ":" << endl;
            for( size_t mb = 0; mb < minibuckets.size(); mb++ )
                cout << "  " << minibuckets[mb] << endl;
        }

        // Process canonical mini-bucket list sequentially, merging the
        // current mini-bucket with a previous one provided that the number
        // of variables in the resulting minibucket does not exceed i
        for( vecBucket::iterator mb = minibuckets.begin(); mb != minibuckets.end(); ) {
            bool merged = false;
            for( vecBucket::iterator mbprev = minibuckets.begin(); mbprev != mb; mbprev++ )
                if( (mb->vars() | mbprev->vars()).size() <= i ) {
                    mbprev->insert( mbprev->end(), mb->begin(), mb->end() );
                    mb = minibuckets.erase( mb );
                    merged = true;
                    break;
                }
            if( !merged )
                mb++;
        }

        if( Verbose() >= 3 ) {
            cout << "Merged minibuckets:" << endl;
            for( size_t mb = 0; mb < minibuckets.size(); mb++ )
                cout << "  " << minibuckets[mb] << endl;
        }

        return minibuckets;
    }


    void IJGP::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa[alpha].fill( 1.0 );
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) 
            _Qb[beta].fill( 1.0 );
        for( size_t e = 0; e < grm().nr_Redges(); e++ )
            _mes[e].fill( 1.0 );
    }


    void IJGP::init( const VarSet &ns ) {
        for( vector<Factor>::iterator alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
            if( alpha->vars() && ns )
                alpha->fill( 1.0 );

        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            if( grm().IR(beta) && ns ) {
                _Qb[beta].fill( 1.0 );
                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ )
                    message( *alpha, beta ).fill( 1.0 );
            }
    }


    string IJGP::identify() const {
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    Factor IJGP::belief( const VarSet &ns ) const {
        vector<Factor>::const_iterator beta;
        for( beta = _Qb.begin(); beta != _Qb.end(); beta++ )
            if( beta->vars() >> ns )
                break;
        if( beta != _Qb.end() )
            return( beta->marginal(ns) );
        else {
            vector<Factor>::const_iterator alpha;
            for( alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
                if( alpha->vars() >> ns )
                    break;
            assert( alpha != _Qa.end() );
            return( alpha->marginal(ns) );
        }
    }


    vector<Factor> IJGP::beliefs() const {
        vector<Factor> result;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            result.push_back( _Qb[beta] );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            result.push_back( _Qa[alpha] );
        return result;
    }


    Factor IJGP::belief( const Var &n ) const {
        return belief( (VarSet)n );
    }


    double IJGP::run() {
        if( Props.verbose >= 1 )
            cout << "Starting " << identify() << "...";
        if( Props.verbose >= 3)
           cout << endl;

        clock_t tic = toc();
        Diffs diffs(grm().nrVars(), 1.0);

        vector<size_t> edge_seq;
//      vector<double> residuals;

        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back( belief( grm().var(i) ) );

/*      if( Props.updates == UpdateType::SEQMAX ) {
            // do the first pass
            for(size_t iI = 0; iI < grm().nrEdges(); iI++ )
                calcNewMessage(iI);

            // calculate initial residuals
            residuals.reserve(grm().nrEdges());
            for( size_t iI = 0; iI < grm().nrEdges(); iI++ )
                residuals.push_back( dist( _newmessages[iI], _messages[iI], Prob::DISTLINF ) );
        } else {*/
            edge_seq.reserve( grm().nr_Redges() );
            for( size_t e = 0; e < grm().nr_Redges(); e++ )
                edge_seq.push_back( e );
//        }

        // IJGP with parent->child message updates
        // Do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) {
/*          if( Props.updates == UpdateType::SEQMAX ) {
                // Residuals-BP by Koller et al.
                for( size_t t = 0; t < grm().nrEdges(); t++ ) {
                    // update the message with the largest residual
                    size_t iI = max_element(residuals.begin(), residuals.end()) - residuals.begin();
                    updateMessage( iI );
                    residuals[iI] = 0;

                    // I->i has been updated, which means that residuals for all
                    // J->j with J in nb[i]\I and j in nb[J]\i have to be updated
                    size_t i = grm().edge(iI).first;
                    size_t I = grm().edge(iI).second;
                    for( FactorGraph::nb_cit J = grm().nbV(i).begin(); J != grm().nbV(i).end(); J++ )
                        if( *J != I )
                            for( FactorGraph::nb_cit j = grm().nbF(*J).begin(); j != grm().nbF(*J).end(); j++ )
                                if( *j != i ) {
                                    size_t jJ = grm().edge(*j,*J);
                                    calcNewMessage(jJ);
                                    residuals[jJ] = dist( _newmessages[jJ], _messages[jJ], Prob::DISTLINF );
                                }
                }
            } else if( Props.updates == UpdateType::PARALL ) {
                // Parallel updates 
                for( size_t t = 0; t < grm().nr_Redges(); t++ )
                    calcNewMessage(t);

                for( size_t t = 0; t < grm().nr_Redges(); t++ )
                    updateMessage(t);
            } else {*/
                // Sequential updates
                if( Props.updates == UpdateType::SEQRND )
                    random_shuffle( edge_seq.begin(), edge_seq.end() );

                for( size_t t = 0; t < grm().nr_Redges(); t++ )
                    updateMessage( edge_seq[t] );
//            }

            // Calc outer region beliefs
            for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
                _Qa[alpha] = grm().OR(alpha);
                for( RegionGraph::R_nb_cit beta2 = grm().nbOR(alpha).begin(); beta2 != grm().nbOR(alpha).end(); beta2++ )
                    for( RegionGraph::R_nb_cit alpha2 = grm().nbIR(*beta2).begin(); alpha2 != grm().nbIR(*beta2).end(); alpha2++ )
                        if( *alpha2 != alpha )
                            _Qa[alpha] *= message(*alpha2, *beta2);
                _Qa[alpha].normalize();
            }

            // Calc inner region beliefs
            for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) {
                _Qb[beta] = Factor();
                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ )
                    _Qb[beta] *= message(*alpha, beta);
                _Qb[beta].normalize();
            }

            // Calculate new variable beliefs and compare with old ones
            for( size_t i = 0; i < grm().nrVars(); i++ ) {
                Factor nb( belief( grm().var(i) ) );
                diffs.push( dist( nb, old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = nb;
            }

            if( Props.verbose >= 3 )
                cout << Name << "::run:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;
        }

        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Props.verbose >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Props.verbose == 1 )
                    cout << endl;
                    cout << Name << "::run:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Props.verbose >= 3 )
                    cout << Name << "::run:  converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


    void IJGP::updateMessage(size_t e) {
        size_t alpha = grm().Redge(e).first;
        size_t beta = grm().Redge(e).second;
        
        Factor prod = grm().OR(alpha);
        for( RegionGraph::R_nb_cit beta2 = grm().nbOR(alpha).begin(); beta2 != grm().nbOR(alpha).end(); beta2++ )
            if( *beta2 != beta )
                for( RegionGraph::R_nb_cit alpha2 = grm().nbIR(*beta2).begin(); alpha2 != grm().nbIR(*beta2).end(); alpha2++ )
                    if( *alpha2 != alpha )
                        prod *= message(*alpha2, *beta2);
        Factor newmessage = prod.marginal( grm().IR(beta) );

        if( Props.damping == 0.0 )
            message(alpha, beta) = newmessage;
        else
            message(alpha, beta) = (message(alpha,beta) ^ Props.damping) * (newmessage ^ (1.0 - Props.damping));
    }


    Complex IJGP::logZ() const {
        Complex sum = 0.0;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            sum += Complex(grm().IR(beta).c()) * _Qb[beta].entropy();
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
            sum += Complex(grm().OR(alpha).c()) * _Qa[alpha].entropy();
            sum += (grm().OR(alpha).log0() * _Qa[alpha]).totalSum();
        }
        return sum;
    }


}
