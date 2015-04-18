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
#include <map>
#include <set>
#include "lclin.h"
#include "diffs.h"
#include "util.h"
#include "alldai.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *LCLin::Name = "LCLIN";


    bool LCLin::initProps() {
        if( !HasProperty("updates") )
            return false;
        if( !HasProperty("cavity") )
            return false;
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        
        Props.updates = FromStringTo<UpdateType>("updates");
        Props.cavity  = FromStringTo<CavityType>("cavity");
        if( HasProperty("reinit") )
            Props.reinit = FromStringTo<bool>("reinit");
        else
            Props.reinit = true;
        Props.tol     = FromStringTo<double>("tol");
        Props.maxiter = FromStringTo<size_t>("maxiter");
        Props.verbose = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            Props.damping = FromStringTo<double>("damping");
        else
            Props.damping = 0.0;
        if (HasProperty("cavainame") )
            Props.cavainame = GetPropertyAs<string>("cavainame");
        if (HasProperty("cavaiopts") )
            Props.cavaiopts = FromStringTo<Properties>("cavaiopts");
        if( HasProperty("cortol") )
            Props.cortol = FromStringTo<double>("cortol");
        else
            Props.cortol = 0.0;

        return true;
    }


    LCLin::LCLin(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _phis(), _gamma(), _beliefs() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        // create phis
        _phis.reserve(grm().nrEdges());
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            size_t i = grm().edge(iI).first;
            size_t I = grm().edge(iI).second;
            _phis.push_back( Factor( grm().factor(I).vars() / grm().var(i) ) );
        }

        // Construct _gamma
        _gamma.resize( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            _gamma[i].push_back(Factor());

        // create beliefs
        _beliefs.reserve( grm().nrVars() );
        for( size_t i=0; i < grm().nrVars(); i++ )
            _beliefs.push_back(Factor(grm().var(i)));
    }


    string LCLin::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    Factor LCLin::Fdelp( size_t i, const Factor &fac ) { 
        std::vector<Factor> facs;
        if( fac.vars().size() )
            facs.push_back( fac );
        std::vector<const Factor *> pfacs;
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ )
            facs.push_back( grm().factor(*I) * phi(i,*I) );
        for( size_t K = 0; K < facs.size(); K++ )
            pfacs.push_back( &(facs[K]) );
        return notSumProd( grm().var(i), pfacs );
    }


    Factor LCLin::Fdel( size_t i ) { 
        std::vector<Factor> facs;
        std::vector<const Factor *> pfacs;
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ )
            facs.push_back( grm().factor(*I) * phi(i,*I) );
        for( size_t K = 0; K < facs.size(); K++ )
            pfacs.push_back( &(facs[K]) );
        return notSumProd( grm().var(i), pfacs );
    }


    Factor LCLin::Gdele( size_t i, size_t I1 ) { 
        std::vector<Factor> facs;
        std::vector<const Factor *> pfacs;
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ )
            if( *I != I1 )
                facs.push_back( grm().factor(*I) * phi(i,*I) );
        for( size_t K = 0; K < facs.size(); K++ )
            pfacs.push_back( &(facs[K]) );
        return notSumProd( grm().factor(I1).vars() / grm().var(i), pfacs );
    }


    Factor LCLin::Gdelpe( size_t i, const Factor &fac, size_t I1 ) { 
        std::vector<Factor> facs;
        if( fac.vars().size() )
            facs.push_back( fac );
        std::vector<const Factor *> pfacs;
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ )
            if( *I != I1 )
                facs.push_back( grm().factor(*I) * phi(i,*I) );
        for( size_t K = 0; K < facs.size(); K++ )
            pfacs.push_back( &(facs[K]) );
        return notSumProd( grm().factor(I1).vars() / grm().var(i), pfacs );
    }


    Factor LCLin::Hdelpe( size_t i, const Factor &fac, size_t Y, VarSet ns ) {
        std::vector<Factor> facs;
        if( fac.vars().size() )
            facs.push_back( fac );
        std::vector<const Factor *> pfacs;
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ )
            if( *I != Y )
                facs.push_back( grm().factor(*I) * phi(i,*I) );
        for( size_t K = 0; K < facs.size(); K++ )
            pfacs.push_back( &(facs[K]) );
        return notSumProd( ns, pfacs );
    }


    double LCLin::CalcCavityDist (size_t i, const std::string &name, const Properties &opts) {
        double maxdiff = 0.0;

        if( Verbose() >= 3 )
            cout << "Initing cavity " << grm().var(i) << "(" << grm().delta(grm().var(i)).size() << " vars, " << grm().delta(grm().var(i)).stateSpace() << " states)" << endl;

        _gamma[i].clear();
        if( Props.cavity == CavityType::UNIFORM )
            _gamma[i].push_back(Factor());
        else if( Props.cavity == CavityType::PAIR || Props.cavity == CavityType::PAIR2 || Props.cavity == CavityType::PAIRFAST || Props.cavity == CavityType::FULL || Props.cavity == CavityType::GROUP || Props.cavity == CavityType::HEUR ) {
            InferenceAlgorithm *cav = newInfAlg( name, grm(), opts );
            cav->grm().makeCavity( grm().var(i) );

            if( Props.cavity == CavityType::FULL ) {
                _gamma[i].push_back( calcMarginal( *cav, grm().delta(grm().var(i)), Props.reinit ).normalized( grm().normType() ) );
                if( Verbose() >= 3 ) {
                    Factor q = _gamma[i].back();
                    Factor q_indep;
                    VarSet ns = q.vars();
                    for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ )
                        q_indep *= q.marginal( *n );
                    cout << "This should be small: " << (q - q_indep).maxAbs() << endl;
                    cout << "Or maybe this:        " << (q - q_indep).divided_by(q_indep).maxAbs() << endl;
                }
                if( Verbose() >= 5 )
                    cout << "Joint: " << _gamma[i].back() << endl;
            } else if( Props.cavity == CavityType::PAIR || Props.cavity == CavityType::PAIR2 || Props.cavity == CavityType::PAIRFAST || Props.cavity == CavityType::HEUR ) {
                vector<Factor> pairbeliefs;
                if( Props.cavity == CavityType::PAIR || Props.cavity == CavityType::HEUR ) 
                    pairbeliefs = calcPairBeliefs( *cav, grm().delta(grm().var(i)), Props.reinit );
                else if( Props.cavity == CavityType::PAIR2 )
                    pairbeliefs = calcPairBeliefsNew( *cav, grm().delta(grm().var(i)), Props.reinit );
                else if( Props.cavity == CavityType::PAIRFAST ) {
                    WeightedGraph<double> gr;
                    for( size_t e = 0; e < grm().nrEdges(); e++ ) {
                        size_t e_i = grm().edge(e).first;
                        size_t e_I = grm().edge(e).second;
                        if( !(grm().factor(e_I).vars() && grm().var(i)) )
                            gr[DEdge(e_i, e_I + grm().nrVars())] = 1.0;
                    }

                    InferenceAlgorithm *cavbp = newInfAlg( string("BP"), grm(), Properties()("updates",string("SEQMAX"))("verbose",string("0"))("tol",string("1e-9"))("maxiter", string("10000")) );
                    cavbp->grm().makeCavity( grm().var(i) );
                    cavbp->init();
                    cavbp->run();

                    VarSet di = grm().delta(grm().var(i));
                    for( VarSet::const_iterator j = di.begin(); j != di.end(); j++ ) {
                        if( Verbose() >= 3 )
                            cout << "Cavity " << grm().var(i) << ", shortest paths to " << *j << ":" << endl;
                        map<size_t,size_t> prev = DijkstraShortestPaths( gr, grm().findVar(*j) );

                        for( VarSet::const_iterator k = j; (++k) != di.end(); ) {
                            size_t u = grm().findVar(*k);

                            if( prev.count(u) == 0 ) {
                                if( Verbose() >= 3 )
                                    cout << "no paths from " << *k << " to " << *j << endl;
                            } else {
                                vector<size_t> path_kj;
                                path_kj.push_back( u );

                                Factor psi_kj = cavbp->belief( grm().var(u) );
                                if( Verbose() >= 4 )
                                    cout << cavbp->belief( grm().var(u) ) << endl;

                                while( prev.count(u) ) {
                                    size_t new_u = prev[u];
                                    if( new_u < grm().nrVars() ) {
                                        psi_kj = psi_kj.notSum( *j | *k | grm().var(new_u) );
                                        if( Verbose() >= 4 )
                                            cout << psi_kj << endl;
                                    } else {
                                        if( Verbose() >= 4 ) {
                                            cout << " ... * " << cavbp->belief( grm().factor(new_u - grm().nrVars()).vars() ) << endl;
                                            cout << " ... / " << cavbp->belief( grm().var(u) ) << endl;
                                        }
                                        psi_kj *= cavbp->belief( grm().factor(new_u - grm().nrVars()).vars() ) / cavbp->belief( grm().var(u) );
                                    }

                                    u = prev[u];
                                    path_kj.push_back( u );
                                }

                                if( Verbose() >= 3 ) {
                                    for( size_t n = 0; n < path_kj.size(); n++ )
                                        cout << path_kj[n] << " ";
                                    cout << endl;
                                    cout << psi_kj.marginal( *k | *j ) << endl;
//                                    cout << calcMarginal( *cav, *k | *j, true ).normalized( grm().normType() ) << endl;
                                }
//                                if( Verbose() >= 2 ) 
//                                    cout << dist( psi_kj.marginal( *k | *j ), calcMarginal( *cav, *k | *j, true ).normalized( grm().normType() ), Prob::DISTL1 ) << endl;
                                pairbeliefs.push_back( psi_kj.marginal( *k | *j ) );
//                                pairbeliefs.push_back( calcMarginal( *cav, *k | *j, true ).normalized( grm().normType() ) );
                            }
                        }
                    }

                    delete cavbp;
                }
                
                VarSet weakvars = grm().delta(grm().var(i));
                if( Props.cavity != CavityType::HEUR )
                    _gamma[i].push_back(Factor());
                for( size_t ind = 0; ind < pairbeliefs.size(); ind++ ) {
                    Factor result;
                    for( VarSet::const_iterator j = pairbeliefs[ind].vars().begin(); j != pairbeliefs[ind].vars().end(); j++ )
                        result *= pairbeliefs[ind].marginal(*j);
                    result = pairbeliefs[ind].normalized().divided_by(result) - 1.0;
                    if( result.maxAbs() > Props.tol ) {
                        result.makeZero( Props.tol );
                        assert( !result.hasNaNs() );
                        if( Props.cavity != CavityType::HEUR ) {
                            _gamma[i].push_back( result );
                            if( Verbose() >= 4 )
                                cout << "gamma(" << grm().var(i) << ") += " << result << endl;
                        } else {
                            bool strong = (result.maxAbs() > Props.cortol);
                            if( strong ) 
                                weakvars /= pairbeliefs[ind].vars();
                            if( Verbose() >= 4 )
                                cout << "found " << (strong ? "strong" : "weak") << " correlation: " << result << endl;
                        }
                    }
                }
                if( Props.cavity == CavityType::HEUR ) {
                    Factor joint = calcMarginal( *cav, grm().delta(grm().var(i)) / weakvars, Props.reinit ).normalized( grm().normType() );
                    if( Verbose() >= 4 ) {
                        cout << "WEAKVARS: " << weakvars << " out of " << grm().delta(grm().var(i)) << endl;
                        cout << "STRONG joint: " << joint << endl;
                    }
                    _gamma[i].push_back( joint );
                }
            } else {
                _gamma[i].push_back(Factor());
                if( Verbose() >= 4 ) {
                    cout << " neighbouring factors of " << grm().var(i) << ": ";
                    for( size_t _J = 0; _J < grm().nbV(i).size(); _J++ )
                        cout << grm().factor(grm().nbV(i)[_J]).vars() << " ";
                    cout << endl;
                }
                for( size_t _J = 0; _J < grm().nbV(i).size() - 1; _J++ )
                    for( size_t _K = _J + 1; _K < grm().nbV(i).size(); _K++ ) {
                        size_t J = grm().nbV(i)[_J];
                        size_t K = grm().nbV(i)[_K];
                        VarSet Jmini = grm().factor(J).vars() / grm().var(i);
                        VarSet Kmini = grm().factor(K).vars() / grm().var(i);
    //                    if( Jmini && Kmini )
    //                        cerr << "WARNING: OVERLAPS PRESENT!" << endl;
                        assert( !(Jmini && Kmini) );
                        if( Jmini.size() != 0 && Kmini.size() != 0 ) {
                            if( Verbose() >= 4 )
                                cout << " J\\i=" << Jmini << ", K\\i=" << Kmini << endl;
                            Factor joint = calcMarginal( *cav, Jmini | Kmini, Props.reinit );
                            if( Verbose() >= 4 ) {
                                cout << " joint = " << joint << endl;
                                cout << " joint.marginal(Jmini) = " << joint.marginal(Jmini).embed( Jmini | Kmini ) << endl;
                                cout << " joint.marginal(Kmini) = " << joint.marginal(Kmini).embed( Jmini | Kmini ) << endl;
                                cout << "  product = " << joint.marginal(Kmini) * joint.marginal(Jmini) << endl;
                                cout << " quotient = " << joint.divided_by( joint.marginal(Jmini) * joint.marginal(Kmini) ) << endl;
                            }
                            Factor result = joint.divided_by( joint.marginal(Jmini) * joint.marginal(Kmini) ) - 1.0;
                            if( Verbose() >= 4 )
                                cout << " result = " << result << endl;
                            if( result.maxAbs() > Props.tol ) {
                                result.makeZero( Props.tol );
                                assert( !result.hasNaNs() );
                                _gamma[i].push_back( result );
                                if( Verbose() >= 4 )
                                    cout << "gamma(" << i << ") += " << result << endl;
                            } else {
                                if( Verbose() >= 4 )
                                    cout << "discarded..." << endl;
                            }
                        }
                    }
            }

            maxdiff = cav->maxDiff();
            delete cav;
        } else
            DAI_THROW(UNKNOWN_PROPERTY_TYPE);

        return maxdiff;
    }


    double LCLin::InitCavityDists (const std::string &name, const Properties &opts) {
        clock_t tic = toc();

        if( Verbose() >= 2 )
            cout << Name << "::InitCavityDists starting..." << endl;

        double maxdiff = 0.0;
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            double md = CalcCavityDist(i, name, opts);
            if( md > maxdiff )
                maxdiff = md;
        }
        init();

        if( Verbose() >= 2 )
            cout << Name << "::InitCavityDists:  used " << toc() - tic << " clocks." << endl;

        return maxdiff;
    }


    void LCLin::init() {
        // Init _phis
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            if( Props.updates == UpdateType::SEQRND )
                _phis[iI].randomize();
            else
                _phis[iI].fill(1.0);
        }

        // Calc initial beliefs
        for( size_t i = 0; i < grm().nrVars(); i++ )
            CalcBelief(i);
    }


    Factor LCLin::notSumProd (VarSet exceptVars, std::vector<const Factor *> facs) {
        // calculates the sum over all except exceptVars of the product 
        // of facs i.e., it attempts to speed up the calculation of
        // SUM{x_{\setminus exceptVars}} PROD{i in facs} *(facs[I])
        
        if( facs.size() == 0 )
            return Factor();
        else {
    /*
#ifdef DEBUG
            Factor finalProdSlow;
            if( Verbose() >= 6 ) 
                cout << "Entering notSumProd. facs:" << endl;
            for( vector<const Factor *>::const_iterator K = facs.begin(); K != facs.end(); K++ ) {
                if( Verbose() >= 6 ) 
                    cout << *(*K) << endl;
                finalProdSlow *= *(*K);
            }
            if( Verbose() >= 6 )
                cout << "summing " << finalProdSlow.vars() << " over everything except " << exceptVars << endl;
            finalProdSlow = finalProdSlow.notSum(exceptVars);
#endif
    */
            VarSet sumVars;
            for( vector<const Factor *>::iterator K = facs.begin(); K != facs.end(); K++ )
                sumVars |= (*K)->vars();
            sumVars /= exceptVars;
            if( Verbose() >= 6 ) 
                cout << "sum over " << sumVars << ", not over " << exceptVars << endl;

            Factor finalProd;
            while( sumVars.size() > 0 ) {
                Var i = *(sumVars.begin());

                Factor part;
                VarSet group = i;
                if( Verbose() >= 6 )
                    cout << "start searching factors involving " << group << endl;
                for( vector<const Factor *>::iterator K = facs.begin(); K != facs.end(); ) {
                    if( (*K)->vars() && group ) {
                        if( Verbose() >= 6 )
                            cout << "   including " << *(*K) << endl;
                        part *= *(*K);
                        group |= (*K)->vars() & sumVars;
                        if( Verbose() >= 6 )
                            cout << " now searching for factors involving " << group << endl;
                        facs.erase(K);
                        K = facs.begin();
                    } else
                        K++;
                }
                if( Verbose() >= 6 ) {
                    cout << "   final group: " << group << endl;
                    cout << "   summing over " << (part.vars() & sumVars) << endl;
                    cout << "   yields:      " << part.notSum(exceptVars) << endl;
                }
                finalProd *= part.notSum(exceptVars);
                sumVars /= part.vars();
            }
            // Don't forget the remaining factors in facts which involve (subsets of) exceptVars
            if( Verbose() >= 6 )
                cout << "multiplying with remaining factors" << endl;
            for( vector<const Factor *>::iterator K = facs.begin(); K != facs.end(); K++ ) {
                assert( (*K)->vars() << exceptVars );
                if( Verbose() >= 6 )
                    cout << "   including " << *(*K) << endl;
                finalProd *= *(*K);
            }

    /*
#ifdef DEBUG
            if( Verbose() >= 6 ) {
                cout << "finalProdSlow: " << finalProdSlow << endl;
                cout << "finalProd: " << finalProd << endl;
                cout << "dist = " << dist( finalProdSlow, finalProd, Prob::DISTTV ) << endl;
            }
            assert( dist( finalProdSlow, finalProd, Prob::DISTTV ) < 1e-09 );
#endif
    */
            return finalProd;
        }
    }


    void LCLin::CalcBelief (size_t i) {
        if( Verbose() >= 4 )
            cout << "Entering CalcBelief(" << i << ")" << endl;
        Factor result(grm().var(i), 0.0);
    //  result = Fdel(i);
        if( Verbose() >= 5 )
            cout << "_gamma[i].size() = " << _gamma[i].size() << ", contents: " << endl;
        for( size_t ind = 0; ind < _gamma[i].size(); ind++ ) {
            if( Verbose() >= 5 )
                cout << "   " << _gamma[i][ind] << endl;
            result += Fdelp(i, _gamma[i][ind]);
        }

        _beliefs[i] = result.normalized();
        if( Verbose() >= 4 ) 
            cout << "_beliefs[" << i << "] = " << _beliefs[i] << endl;
    }


    /// Recalculates phi(i,Y)
    void LCLin::update( size_t i, size_t Y ) {
        VarSet Ymini = grm().factor(Y).vars() / grm().var(i);

        // calc denominator
        if( Verbose() >= 4 )
            cout << "i: " << grm().var(i) << ", Y: " << grm().factor(Y).vars() << endl;
        Factor denom(Ymini, 0.0);
    //  denom = Gdele(i, Y).embed(Ymini);
        for( size_t ind = 0; ind < _gamma[i].size(); ind++ )
            denom += Gdelpe(i, _gamma[i][ind], Y).embed(Ymini);
        if( Verbose() >= 4 )
            cout << "denom: " << denom << endl;
        assert( !denom.hasNaNs() );
        assert( !denom.hasNegatives() );

        // calc numerator
        Factor numer;
        for( FactorGraph::nb_cit j = grm().nbF(Y).begin(); j != grm().nbF(Y).end(); j++ ) 
            if( *j != i ) {
                VarSet Yminj = grm().factor(Y).vars() / grm().var(*j);

                Factor part(Ymini, 0.0);
    //            part = Hdelpe(*j, phi(*j,Y), Y, Ymini).embed(Ymini);
                for( size_t ind = 0; ind < _gamma[*j].size(); ind++ )
                    part += Hdelpe(*j, _gamma[*j][ind] * phi(*j,Y), Y, Ymini).embed(Ymini);
                if( Verbose() >= 5 )
                    cout << "i=" << grm().var(i) << ", part *j=" << grm().var(*j) << ": " << part << endl;
                assert( !part.hasNegatives() );

                numer *= part;
            }
        assert( !numer.hasNaNs() );
        if( Verbose() >= 5 )
            cout << "phi(" << i << "," << Y << ") = " << phi(i,Y) << endl;
        if( Verbose() >= 4 )
            cout << "numer before power: " << numer << endl;
        assert( !numer.hasNegatives() );
        numer ^= 1.0 / (grm().nbF(Y).size() - 1);
        if( Verbose() >= 4 )
            cout << "numer after power: " << numer << endl;
        assert( !numer.hasNaNs() );

        // calc phi
        if( Props.damping == 0.0 )
            phi(i,Y) = numer.divided_by(denom).normalized();
        else {
            Factor oldphi = phi(i,Y).normalized();
            Factor newphi = numer.divided_by(denom).normalized();
            if( Verbose() >= 4 )
                cout << " oldphi = " << oldphi << endl << " newphi = " << newphi << endl;
            phi(i,Y) = ((oldphi^Props.damping) * (newphi^(1.0-Props.damping))).normalized();
        }
        if( Verbose() >= 4 )
            cout << "new phi(i=" << i << ",Y=" << Y << ") = " << phi(i,Y) << endl;
    }


    double LCLin::run() {
        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";
        if( Verbose() >= 2 )
            cout << endl;

        clock_t tic = toc();
        Diffs diffs(grm().nrVars(), 1.0);

        double md = InitCavityDists(Props.cavainame, Props.cavaiopts);
        if( md > _maxdiff )
            _maxdiff = md;

        vector<Factor> old_beliefs;
        for(size_t i=0; i < grm().nrVars(); i++ )
            old_beliefs.push_back(beliefV(i));

        vector<size_t> update_seq(grm().nrEdges(),0);
        for( size_t k=0; k < grm().nrEdges(); k++ )
            update_seq[k] = k;

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) {
            // Sequential updates
            if( Props.updates == UpdateType::SEQRND )
                random_shuffle( update_seq.begin(), update_seq.end() );
            
            for( size_t t=0; t < grm().nrEdges(); t++ ) {
                FactorGraph::edge_type iI = grm().edge(update_seq[t]);
                update( iI.first, iI.second );
            }

            // compare new beliefs with old ones
            for(size_t i=0; i < grm().nrVars(); i++ ) {
                CalcBelief(i);
                diffs.push( dist( beliefV(i), old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = beliefV(i);
            }

            if( Verbose() >= 3 )
                cout << Name << "::run:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;
        }

        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Verbose() >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Verbose() == 1 )
                    cout << endl;
                    cout << Name << "::run:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Verbose() >= 2 )
                    cout << Name << "::run:  ";
                    cout << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


}
