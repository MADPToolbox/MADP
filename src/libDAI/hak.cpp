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


#include <map>
#include "hak.h"
#include "util.h"
#include "diffs.h"
#include "exceptions.h"

//#include <fenv.h>


namespace libDAI {


    using namespace std;


    const char *HAK::Name = "HAK";


    bool HAK::initProps() {
        if( !HasProperty("clusters") )
            return false;
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        if( !HasProperty("doubleloop") )
            return false;
        
        Props.clusters  = FromStringTo<ClustersType>("clusters");
        Props.tol       = FromStringTo<double>("tol");
        Props.maxiter   = FromStringTo<size_t>("maxiter");
        Props.verbose   = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            Props.damping = FromStringTo<double>("damping");
        else
            Props.damping = 0.0;
        Props.doubleloop = FromStringTo<bool>("doubleloop");

        if( HasProperty("loopdepth") )
            Props.loopdepth = FromStringTo<size_t>("loopdepth");
        else if( Props.clusters == ClustersType::LOOP )
            return false;

        return true;
    }


    void HAK::constructMessages() {
        // Create outer beliefs
        _Qa.clear();
        _Qa.reserve(grm().nr_ORs());
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            _Qa.push_back( Factor( grm().OR(alpha).vars() ) );

        // Create inner beliefs
        _Qb.clear();
        _Qb.reserve(grm().nr_IRs());
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            _Qb.push_back( Factor( grm().IR(beta) ) );
        
        // Create messages
        _muab.clear();
        _muab.reserve(grm().nr_Redges());
        _muba.clear();
        _muba.reserve(grm().nr_Redges());
        for( vector<RegionGraph::R_edge_t>::const_iterator ab = grm().Redges().begin(); ab != grm().Redges().end(); ab++ ) {
            _muab.push_back( Factor( grm().IR(ab->second) ) );
            _muba.push_back( Factor( grm().IR(ab->second) ) );
        }
    }


    HAK::HAK(const RegionGraph & rg, const Properties &opts) : DAIAlgRG(rg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _Qa(), _Qb(), _muab(), _muba() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        constructMessages();
    }


    void HAK::findLoopClusters( const FactorGraph & fg, std::set<VarSet> &allcl, VarSet newcl, const Var & root, size_t length, VarSet vars ) {
        for( VarSet::const_iterator in = vars.begin(); in != vars.end(); in++ ) {
            VarSet ind = fg.delta( *in );
            if( (newcl.size()) >= 2 && (ind >> root) ) {
                allcl.insert( newcl | *in );
            }
            else if( length > 1 )
                findLoopClusters( fg, allcl, newcl | *in, root, length - 1, ind / newcl );
        }
    }


    HAK::HAK(const FactorGraph & fg, const Properties &opts) : DAIAlgRG(opts), Props(), _maxdiff(0.0), _iterations(0UL), _Qa(), _Qb(), _muab(), _muba() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        vector<VarSet> cl;
        if( Props.clusters == ClustersType::MIN ) {
            cl = fg.Cliques();
        } else if( Props.clusters == ClustersType::DELTA ) {
            for( size_t i = 0; i < fg.nrVars(); i++ )
                cl.push_back(fg.Delta(fg.var(i))); 
        } else if( Props.clusters == ClustersType::LOOP ) {
            cl = fg.Cliques();
            set<VarSet> scl;
            for( vector<Var>::const_iterator i0 = fg.vars().begin(); i0 != fg.vars().end(); i0++ ) {
                VarSet i0d = fg.delta(*i0);
                if( Props.loopdepth > 1 )
                    findLoopClusters( fg, scl, *i0, *i0, Props.loopdepth - 1, fg.delta(*i0) );
            }
            for( set<VarSet>::const_iterator c = scl.begin(); c != scl.end(); c++ )
                cl.push_back(*c);
            if( Verbose() >= 3 ) {
                cout << "HAK uses the following clusters: " << endl;
                for( vector<VarSet>::const_iterator cli = cl.begin(); cli != cl.end(); cli++ )
                    cout << *cli << endl;
            }
        } else
            DAI_THROW(UNKNOWN_PROPERTY_TYPE);

        RegionGraph rg(fg,cl);
        grm() = rg;
        constructMessages();

    /*  if( (Props.clusters == ClustersType::LOOP) ) {
             double c_sum = 0.0;
             for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
                 c_sum += grm().OR(alpha).c();
             for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
                 c_sum += grm().IR(beta).c();
             cout << "Sum of cluster numbers: " << c_sum << endl;
        }*/

        if( Verbose() >= 3 )
            cout << "HAK regiongraph: " << grm() << endl;
    }


    std::string HAK::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void HAK::init( const VarSet &ns ) {
        for( vector<Factor>::iterator alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
            if( alpha->vars() && ns )
                alpha->fill( 1.0 / alpha->stateSpace() );

        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            if( grm().IR(beta) && ns ) {
                _Qb[beta].fill( 1.0 / grm().IR(beta).stateSpace() );
                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ ) {
                    muab(*alpha,beta).fill( 1.0 / grm().IR(beta).stateSpace() );
                    muba(beta,*alpha).fill( 1.0 / grm().IR(beta).stateSpace() );
                }
            }
    }


    void HAK::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        for( vector<Factor>::iterator alpha = _Qa.begin(); alpha != _Qa.end(); alpha++ )
            alpha->fill( 1.0 / alpha->stateSpace() );

        for( vector<Factor>::iterator beta = _Qb.begin(); beta != _Qb.end(); beta++ )
            beta->fill( 1.0 / beta->stateSpace() );

        for( size_t ab = 0; ab < grm().nr_Redges(); ab++ ) {
            _muab[ab].fill( 1.0 / _muab[ab].stateSpace() );
            _muba[ab].fill( 1.0 / _muba[ab].stateSpace() );
        }
    }


    double HAK::doGBP() {
        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";
        if( Verbose() >= 3)
            cout << endl;

// TODO: FIND OUT WHETHER IT WOULD BE USEFUL TO CATCH NANS BY TRAPPING FLOATING POINT EXCEPTIONS
//      feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW | FE_UNDERFLOW);

        clock_t tic = toc();

        // Check whether counting numbers won't lead to problems
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            assert( grm().nbIR(beta).size() + grm().IR(beta).c() != 0.0 );

        // Keep old beliefs to check convergence
        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back( belief( grm().var(i) ) );

        // Differences in single node beliefs
        Diffs diffs(grm().nrVars(), 1.0);

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) {
            for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) {
                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ ) {
                    muab(*alpha,beta) = _Qa[*alpha].marginal(grm().IR(beta)).divided_by( muba(beta,*alpha) );
                    /* TODO: INVESTIGATE THIS PROBLEM
                     *
                     * In some cases, the muab's can have very large entries because the muba's have very
                     * small entries. This may cause NANs later on (e.g., multiplying large quantities may
                     * result in +inf; normalization then tries to calculate inf / inf which is NAN). 
                     * A fix of this problem would consist in normalizing the messages muab.
                     * However, it is not obvious whether this is a real solution, because it has a
                     * negative performance impact and the NAN's seem to be a symptom of a fundamental
                     * numerical unstability.
                     */
                     muab(*alpha,beta).normalize( grm().normType() ); 
                }

                Factor Qb_new;
                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ )
                    Qb_new *= muab(*alpha,beta) ^ (1 / (grm().nbIR(beta).size() + grm().IR(beta).c()));
                Qb_new.normalize( grm().normType() );
                if( Qb_new.hasNaNs() ) {
                    // TODO: WHAT TO DO IN THIS CASE?
                    cout << "HAK::doGBP:  Qb_new has NaNs!" << endl;
                    return NAN;
                }
                /* TODO: WHAT IS THE PURPOSE OF THE FOLLOWING CODE?
                 *
                 *   _Qb[beta] = Qb_new.makeZero(1e-100);
                 */

                if( Props.doubleloop || Props.damping == 0.0 )
                    _Qb[beta] = Qb_new; // no damping for double loop
                else
                    _Qb[beta] = (Qb_new^(1.0 - Props.damping)) * (_Qb[beta]^Props.damping);

                for( RegionGraph::R_nb_cit alpha = grm().nbIR(beta).begin(); alpha != grm().nbIR(beta).end(); alpha++ ) {
                    muba(beta,*alpha) = _Qb[beta].divided_by( muab(*alpha,beta) );

                    /* TODO: INVESTIGATE WHETHER THIS HACK (INVENTED BY KEES) TO PREVENT NANS MAKES SENSE 
                     *
                     *   muba(beta,*alpha).makePositive(1e-100);
                     *
                     */

                    Factor Qa_new = grm().OR(*alpha);
                    for( RegionGraph::R_nb_cit gamma = grm().nbOR(*alpha).begin(); gamma != grm().nbOR(*alpha).end(); gamma++ )
                        Qa_new *= muba(*gamma,*alpha);
                    Qa_new ^= (1.0 / grm().OR(*alpha).c());
                    Qa_new.normalize( grm().normType() );
                    if( Qa_new.hasNaNs() ) {
                        cout << "HAK::doGBP:  Qa_new has NaNs!" << endl;
                        return NAN;
                    }
                    /* TODO: WHAT IS THE PURPOSE OF THE FOLLOWING CODE?
                     *
                     *   _Qb[beta] = Qb_new.makeZero(1e-100);
                     */

                if( Props.doubleloop || Props.damping == 0.0 )
                    _Qa[*alpha] = Qa_new; // no damping for double loop
                else
                    // FIXME: GEOMETRIC DAMPING IS SLOW!
                    _Qa[*alpha] = (Qa_new^(1.0 - Props.damping)) * (_Qa[*alpha]^Props.damping);
                }
            }

            // Calculate new single variable beliefs and compare with old ones
            for( size_t i = 0; i < grm().nrVars(); i++ ) {
                Factor new_belief = belief( grm().var( i ) );
                diffs.push( dist( new_belief, old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = new_belief;
            }

            if( Verbose() >= 3 )
                cout << "HAK::doGBP:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;
        }

        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Verbose() >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Verbose() == 1 )
                    cout << endl;
                cout << "HAK::doGBP:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Verbose() >= 2 )
                    cout << "HAK::doGBP:  ";
                cout << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


    double HAK::doDoubleLoop() {
        if( Verbose() >= 1 )
            cout << "Starting " << identify() << "...";
        if( Verbose() >= 3)
            cout << endl;

        clock_t tic = toc();

        // Save original outer regions
        vector<FRegion> org_ORs = grm().ORs();

        // Save original inner counting numbers and set negative counting numbers to zero
        vector<double> org_IR_cs( grm().nr_IRs(), 0.0 );
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ ) {
            org_IR_cs[beta] = grm().IR(beta).c();
            if( grm().IR(beta).c() < 0.0 )
                grm().IR(beta).c() = 0.0;
        }

        // Keep old beliefs to check convergence
        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back( belief( grm().var(i) ) );

        // Differences in single node beliefs
        Diffs diffs(grm().nrVars(), 1.0);

        size_t outer_maxiter   = Props.maxiter;
        double outer_tol       = Props.tol;
        size_t outer_verbose   = Verbose();
        double org_maxdiff     = _maxdiff;

        // Set parameters for inner loop
        Props.maxiter = 5;
        Props.verbose = outer_verbose ? outer_verbose - 1 : 0;

        size_t outer_iter = 0;
        size_t total_iter = 0;
        for( outer_iter = 0; total_iter < outer_maxiter && diffs.max() > outer_tol; outer_iter++ ) {
            // Calculate new outer regions
            for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
                grm().OR(alpha) = org_ORs[alpha];
                for( RegionGraph::R_nb_cit beta = grm().nbOR(alpha).begin(); beta != grm().nbOR(alpha).end(); beta++ )
                    grm().OR(alpha) *= _Qb[*beta] ^ ((grm().IR(*beta).c() - org_IR_cs[*beta]) / grm().nbIR(*beta).size());
            }

            // Inner loop
            if( std::isnan( doGBP() ) )
                return NAN;

            // Calculate new single variable beliefs and compare with old ones
            for( size_t i = 0; i < grm().nrVars(); i++ ) {
                Factor new_belief = belief( grm().var( i ) );
                diffs.push( dist( new_belief, old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = new_belief;
            }

            total_iter += Iterations();

            if( Verbose() >= 3 )
                cout << "HAK::doDoubleLoop:  maxdiff " << diffs.max() << " after " << total_iter << " passes" << endl;
        }

        // restore _maxiter, _verbose and _maxdiff
        Props.maxiter = outer_maxiter;
        Props.verbose = outer_verbose;
        _maxdiff = org_maxdiff;

        _iterations = total_iter;
        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        // Restore original outer regions
        grm().ORs() = org_ORs;

        // Restore original inner counting numbers
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            grm().IR(beta).c() = org_IR_cs[beta];

        if( Verbose() >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Verbose() == 1 )
                    cout << endl;
                    cout << "HAK::doDoubleLoop:  WARNING: not converged within " << outer_maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
                } else {
                    if( Verbose() >= 3 )
                        cout << "HAK::doDoubleLoop:  ";
                    cout << "converged in " << total_iter << " passes (" << toc() - tic << " clocks)." << endl;
                }
            }

        return diffs.max();
    }


    double HAK::run() {
        if( Props.doubleloop )
            return doDoubleLoop();
        else
            return doGBP();
    }


    Factor HAK::belief( const VarSet &ns ) const {
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


    Factor HAK::belief( const Var &n ) const {
        return belief( (VarSet)n );
    }


    vector<Factor> HAK::beliefs() const {
        vector<Factor> result;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            result.push_back( Qb(beta) );
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ )
            result.push_back( Qa(alpha) );
        return result;
    }


    Complex HAK::logZ() const {
        Complex sum = 0.0;
        for( size_t beta = 0; beta < grm().nr_IRs(); beta++ )
            sum += Complex(grm().IR(beta).c()) * Qb(beta).entropy();
        for( size_t alpha = 0; alpha < grm().nr_ORs(); alpha++ ) {
            sum += Complex(grm().OR(alpha).c()) * Qa(alpha).entropy();
            sum += (grm().OR(alpha).log0() * Qa(alpha)).totalSum();
        }
        return sum;
    }


}
