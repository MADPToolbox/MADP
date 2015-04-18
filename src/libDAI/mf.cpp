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
#include <sstream>
#include <map>
#include <set>
#include "mf.h"
#include "diffs.h"
#include "util.h"
#include "exceptions.h"


namespace libDAI {



    using namespace std;


    const char *MF::Name = "MF";


    bool MF::initProps() {
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        
        Props.tol     = FromStringTo<double>("tol");
        Props.maxiter = FromStringTo<size_t>("maxiter");
        Props.verbose = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            Props.damping = FromStringTo<double>("damping");
        else
            Props.damping = 0.0;

        return true;
    }


    MF::MF(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts),  Props(), _maxdiff(0.0), _iterations(0UL), _beliefs() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        // clear beliefs
        _beliefs.clear();
        _beliefs.reserve( grm().nrVars() );

        // create beliefs
        for( vector<Var>::const_iterator i = grm().vars().begin(); i != grm().vars().end(); i++ ) 
            _beliefs.push_back(Factor(*i));
    }


    string MF::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void MF::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        for( vector<Factor>::iterator qi = _beliefs.begin(); qi != _beliefs.end(); qi++ )
            qi->fill(1.0);
    }


    double MF::run() {
        clock_t tic = toc();

        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";

        size_t pass_size = _beliefs.size();
        Diffs diffs(pass_size * 3, 1.0);

        size_t t=0;
        for( t=0; t < (Props.maxiter*pass_size) && diffs.max() > Props.tol; t++ ) {
            // choose random Var i
            size_t i = (size_t) (grm().nrVars() * rnd_uniform());

            Factor jan;
            Factor piet;
            for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) {

                Factor henk;
                for( FactorGraph::nb_cit j = grm().nbF(*I).begin(); j != grm().nbF(*I).end(); j++ ) // for all j in I \ i
                    if( *j != i )
                        henk *= _beliefs[*j];
                piet = grm().factor(*I).log0();
                piet *= henk;
                piet = piet.partSum(grm().var(i));
                piet = piet.exp();
                jan *= piet; 
            }
                
            jan.normalize( grm().normType() );

            if( jan.hasNaNs() ) {
                cout << "MF::run():  ERROR: jan has NaNs!" << endl;
                return NAN;
            }

            if( Props.damping != 0.0 )
                jan = (jan^(1.0 - Props.damping)) * (_beliefs[i]^Props.damping);
            diffs.push( dist( jan, _beliefs[i], Prob::DISTLINF ) );

            _beliefs[i] = jan;
        }

        _iterations = t / pass_size;
        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Verbose() >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Verbose() == 1 )
                    cout << endl;
                cout << "MF::run:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Verbose() >= 2 )
                    cout << "MF::run:  ";
                cout << "converged in " << t / pass_size << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


    Factor MF::beliefV( size_t i ) const {
        Factor piet;
        piet = _beliefs[i];
        piet.normalize();
        return(piet);
    }


    Factor MF::belief (const VarSet &ns) const {
        if( ns.size() == 1 )
            return belief( *(ns.begin()) );
        else {
            assert( ns.size() == 1 );
            return Factor();
        }
    }


    Factor MF::belief (const Var &n) const {
        return( beliefV( grm().findVar( n) ) );
    }


    vector<Factor> MF::beliefs() const {
        vector<Factor> result;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            result.push_back( beliefV(i) );
        return result;
    }


    Complex MF::logZ() const {
        Complex sum = 0.0;
        
        for(size_t i=0; i < grm().nrVars(); i++ )
            sum -= beliefV(i).entropy();
        for(size_t I=0; I < grm().nrFactors(); I++ ) {
            Factor henk;
            for( FactorGraph::nb_cit j = grm().nbF(I).begin(); j != grm().nbF(I).end(); j++ )   // for all j in I
                henk *= _beliefs[*j];
            henk.normalize();
            Factor piet;
            piet = grm().factor(I).log0();
            piet *= henk;
            sum -= Complex( piet.totalSum() );
        }

        return -sum;
    }


    void MF::init( const VarSet &ns ) {
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            if( ns && grm().var(i) )
                _beliefs[i].fill( 1.0 );
        }
    }


}
