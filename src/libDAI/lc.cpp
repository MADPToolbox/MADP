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
#include "lc.h"
#include "diffs.h"
#include "util.h"
#include "alldai.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *LC::Name = "LC";


    bool LC::initProps() {
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

        return true;
    }


    LC::LC(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _pancakes(), _cavitydists(), _phis(), _beliefs() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        // create pancakes
        _pancakes.resize(grm().nrVars());
       
        // create cavitydists
        for( size_t i=0; i < grm().nrVars(); i++ )
            _cavitydists.push_back(Factor(grm().delta(grm().var(i))));

        // create phis
        _phis.reserve(grm().nrEdges());
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            size_t i = grm().edge(iI).first;
            size_t I = grm().edge(iI).second;
            _phis.push_back( Factor( grm().factor(I).vars() / grm().var(i) ) );
        }

        // create beliefs
        _beliefs.reserve( grm().nrVars() );
        for( size_t i=0; i < grm().nrVars(); i++ )
            _beliefs.push_back(Factor(grm().var(i)));
    }


    string LC::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void LC::CalcBelief (size_t i) {
        _beliefs[i] = _pancakes[i].marginal(grm().var(i));
    }


    double LC::CalcCavityDist (size_t i, const std::string &name, const Properties &opts) {
        Factor Bi;
        double maxdiff = 0;

        if( Verbose() >= 2 )
            cout << "Initing cavity " << grm().var(i) << "(" << grm().delta(grm().var(i)).size() << " vars, " << grm().delta(grm().var(i)).stateSpace() << " states)" << endl;

        if( Props.cavity == CavityType::UNIFORM )
            Bi = Factor(grm().delta(grm().var(i)));
/*        else if( Props.cavity == CavityType::FULLCLUSTER ) {
            vector<Factor> facs;
            Var vi = grm().var(i);
            VarSet di = grm().delta(vi);
            VarSet Di = grm().Delta(vi);

            // construct new supervariable out of di
            long label = 0;
            for( size_t j = 0; j < grm().nrVars(); j++ )
                if( grm().var(j).label() >= label )
                    label = grm().var(j).label() + 1;
            Var new_di( label, di.stateSpace() );
            if( Verbose() >= 3 )
                cout << "new supervar " << new_di << " instead of " << di << endl;

            for( size_t I = 0; I < grm().nrFactors(); I++ ) {
                VarSet Ivars = grm().factor(I).vars();
                if( !(Ivars && Di) ) {
                    facs.push_back( grm().factor(I) );
                    if( Verbose() >= 3 )
                        cout << Ivars << endl;
                } else if( !(Ivars >> vi) ) {
                    Factor tmpI = grm().factor(I).embed( Ivars | di );
                    vector<double> values;
                    for( size_t di_state = 0; di_state < di.stateSpace(); di_state++ ) {
                        // copy slice
                        Prob sl = tmpI.slice( di, di_state ).p();
                        values.insert( values.end(), sl.p().begin(), sl.p().end() );
                    }
                    facs.push_back( Factor((Ivars / di) | new_di, &(values[0])) );
                    if( Verbose() >= 3 )
                        cout << "super: " << ((Ivars / di) | new_di) << endl;
                } else {
                    if( Verbose() >= 3 )
                        cout << "dropping " << Ivars << endl;
                }
            }
            FactorGraph fg(facs);
            InferenceAlgorithm *cav = newInfAlg( name, fg, opts );
            cav->init();
            cav->run();
            Prob Bivalues = cav->belief( new_di ).p();
            Bi = Factor( di, Bivalues );
            if( Verbose() >= 2 ) {
                cout << Bi << endl;

                InferenceAlgorithm *cav2 = newInfAlg( name, grm(), opts );
                cav2->grm().makeCavity( vi );
                cav2->init();
                cav2->run();
                Factor Bi2;
                for( VarSet::const_iterator j = di.begin(); j != di.end(); j++ )
                    Bi2 *= cav2->belief( *j );

                cout << Bi2 << endl;
            }
        } */
        else {
            InferenceAlgorithm *cav = newInfAlg( name, grm(), opts );
            cav->grm().makeCavity( grm().var(i) );

            if( Props.cavity == CavityType::FULL )
                Bi = calcMarginal( *cav, grm().delta(grm().var(i)), Props.reinit );
            else if( Props.cavity == CavityType::PAIR )
                Bi = calcMarginal2ndO( *cav, grm().delta(grm().var(i)), Props.reinit );
            else if( Props.cavity == CavityType::PAIR2 ) {
                vector<Factor> pairbeliefs = calcPairBeliefsNew( *cav, grm().delta(grm().var(i)), Props.reinit );
                for( size_t ij = 0; ij < pairbeliefs.size(); ij++ )
                    Bi *= pairbeliefs[ij];
            }
            maxdiff = cav->maxDiff();
            delete cav;
        }
        Bi.normalize( grm().normType() );
        _cavitydists[i] = Bi;

        return maxdiff;
    }


    double LC::InitCavityDists (const std::string &name, const Properties &opts) {
        clock_t tic = toc();

        if( Verbose() >= 1 ) {
            cout << Name << "::InitCavityDists:  ";
            if( Props.cavity == CavityType::UNIFORM )
                cout << "Using uniform initial cavity distributions" << endl;
            else if( Props.cavity == CavityType::FULL )
                cout << "Using full " << name << opts << "...";
            else if( Props.cavity == CavityType::PAIR )
                cout << "Using pairwise " << name << opts << "...";
            else if( Props.cavity == CavityType::PAIR2 )
                cout << "Using pairwise(new) " << name << opts << "...";
        }

        double maxdiff = 0.0;
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            double md = CalcCavityDist(i, name, opts);
            if( md > maxdiff )
                maxdiff = md;
        }
        init();

        if( Verbose() >= 1 ) {
            cout << Name << "::InitCavityDists used " << toc() - tic << " clocks." << endl;
        }

        return maxdiff;
    }


    long LC::SetCavityDists (std::vector<Factor> &Q) {
        if( Verbose() >= 1 ) 
            cout << Name << "::SetCavityDists:  Setting initial cavity distributions" << endl;
        if( Q.size() != grm().nrVars() )
            return -1;
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            if( _cavitydists[i].vars() != Q[i].vars() ) {
                return i+1;
            } else
                _cavitydists[i] = Q[i];
        }
        init();
        return 0;
    }


    void LC::init() {
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            if( Props.updates == UpdateType::SEQRND )
                _phis[iI].randomize();
            else
                _phis[iI].fill(1.0);
        }
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            _pancakes[i] = _cavitydists[i];
            
            for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) {
                _pancakes[i] *= grm().factor(*I);
                if( Props.updates == UpdateType::SEQRND )
                  _pancakes[i] *= _phis[grm().edge(i,*I)];
            }
            
            _pancakes[i].normalize( grm().normType() );

            CalcBelief(i);
        }
    }


    Factor LC::NewPancake (size_t iI, bool & hasNaNs) {
        size_t i = grm().edge(iI).first;
        size_t I = grm().edge(iI).second;

        Factor piet = _pancakes[i];

        // recalculate _pancake[i]
        VarSet Ivars = grm().factor(I).vars();
        Factor A_I;
        for( VarSet::const_iterator k = Ivars.begin(); k != Ivars.end(); k++ )
            if( grm().var(i) != *k )
                A_I *= (_pancakes[grm().findVar(*k)] * grm().factor(I).inverse()).partSum( Ivars / grm().var(i) );
        if( Ivars.size() > 1 )
            A_I ^= (1.0 / (Ivars.size() - 1));
        Factor A_Ii = (_pancakes[i] * grm().factor(I).inverse() * _phis[iI].inverse()).partSum( Ivars / grm().var(i) );
        Factor quot = A_I.divided_by(A_Ii);
        if( Props.damping != 0.0 )
            quot = (quot^(1.0 - Props.damping)) * (_phis[iI]^Props.damping);

        piet *= quot.divided_by( _phis[iI] ).normalized( grm().normType() );
        _phis[iI] = quot.normalized( grm().normType() );

        piet.normalize( grm().normType() );

        if( piet.hasNaNs() ) {
            cout << Name << "::NewPancake(" << iI << "):  has NaNs!" << endl;
            hasNaNs = true;
        }

        return piet;
    }


    double LC::run() {
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
            old_beliefs.push_back(belief(i));

        bool hasNaNs = false;
        for( size_t i=0; i < grm().nrVars(); i++ )
            if( _pancakes[i].hasNaNs() ) {
                hasNaNs = true;
                break;
            }
        if( hasNaNs ) {
            cout << Name << "::run:  initial _pancakes has NaNs!" << endl;
            return NAN;
        }

        vector<long> update_seq(grm().nrEdges(),0);
        for( size_t k=0; k < grm().nrEdges(); k++ )
            update_seq[k] = k;

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) {
            // Sequential updates
            if( Props.updates == UpdateType::SEQRND )
                random_shuffle( update_seq.begin(), update_seq.end() );
            
            for( size_t t=0; t < grm().nrEdges(); t++ ) {
                long iI = update_seq[t];
                long i = grm().edge(iI).first;
                _pancakes[i] = NewPancake(iI, hasNaNs);
                if( hasNaNs )
                    return NAN;
                CalcBelief(i);
            }

            // compare new beliefs with old ones
            for(size_t i=0; i < grm().nrVars(); i++ ) {
                diffs.push( dist( belief(i), old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = belief(i);
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
