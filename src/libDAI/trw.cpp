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
#include <algorithm>
#include "trw.h"
#include "diffs.h"
#include "util.h"
#include "properties.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *TRW::Name = "TRW";


    bool TRW::initProps() {
        if( !HasProperty("updates") )
            return false;
        if( !HasProperty("tol") )
            return false;
        if( !HasProperty("outertol") )
            return false;
        if( !HasProperty("maxiter") )
            return false;
        if( !HasProperty("maxouteriter") )
            return false;
        if( !HasProperty("verbose") )
            return false;
        
        Props.updates      = FromStringTo<UpdateType>("updates");
        Props.tol          = FromStringTo<double>("tol");
        Props.outertol     = FromStringTo<double>("outertol");
        Props.maxiter      = FromStringTo<size_t>("maxiter");
        Props.maxouteriter = FromStringTo<size_t>("maxouteriter");
        Props.verbose      = FromStringTo<size_t>("verbose");
        if( HasProperty("damping") )
            Props.damping = FromStringTo<double>("damping");
        else
            Props.damping = 0.0;

        return true;
    }


    TRW::TRW(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _messages(), _newmessages(), _rho_e(), _rho_e_descent() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        // clear messages
        _messages.clear();
        _messages.reserve(grm().nrEdges());

        // create messages
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            size_t i = grm().edge(iI).first;
            _messages.push_back( Prob( grm().var(i).states() ) );
        }

        // create new_messages
        _newmessages = _messages;

        // create valid initial rho_e
        init_rho_e();
    }


    void TRW::init_rho_e() {
        // count number of edges
        size_t E = 0;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            for( size_t j = i+1; j < grm().nrVars(); j++ )
                if( grm().delta(grm().var(i)) && grm().var(j) )
                    E++;

        // init uniform rho_e
        double rho = (grm().nrVars() - 1.0) / E;
//        double rho = 1.0;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            for( size_t j = i+1; j < grm().nrVars(); j++ )
                if( grm().delta(grm().var(i)) && grm().var(j) )
                    _rho_e[UEdge(i,j)] = rho;
    }


    double TRW::calc_rho_descent() {
        WeightedGraph<double> wg;
        // construct weighted graph with as weights the mutual information
        // according to the beliefs and take the maximum spanning tree to
        // find the descent direction
        double maxWeight = 0.0;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            for( size_t j = i+1; j < grm().nrVars(); j++ ) {
                Var v_i = grm().var(i);
                Var v_j = grm().var(j);
                if( grm().delta(v_i) && v_j ) {
                    double w = MutualInfo( belief( v_i | v_j ) );
                    if( w > maxWeight )
                        maxWeight = w;
                    wg[UEdge(i,j)] = -w;
                }
            }
        // Add constant because BGL cannot handle negative weights
        for( WeightedGraph<double>::iterator e = wg.begin(); e != wg.end(); e++ )
            e->second += maxWeight;
            
        DEdgeVec MST = MinSpanningTreePrims( wg );

        for( size_t i = 0; i < grm().nrVars(); i++ )
            for( size_t j = i+1; j < grm().nrVars(); j++ )
                if( grm().delta(grm().var(i)) && grm().var(j) )
                    _rho_e_descent[UEdge(i,j)] = 0.0;
        double MinSpanningTreeWeight = 0.0;
        for( DEdgeVec::const_iterator e = MST.begin(); e != MST.end(); e++ ) {
            MinSpanningTreeWeight -= (wg[UEdge(*e)] - maxWeight);
            _rho_e_descent[UEdge(*e)] = 1.0;
        }

        assert( MinSpanningTreeWeight > 0.0 );
        return MinSpanningTreeWeight;
    }


    void TRW::update_rho_e(double alpha) {
        for( WeightedGraph<double>::iterator ij = _rho_e.begin(); ij != _rho_e.end(); ij++ )
            ij->second = (1.0 - alpha) * (ij->second) + alpha * _rho_e_descent[ij->first];
    }


    void TRW::init() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        init_messages();
        init_rho_e();
    }

    void TRW::init_messages() {
        for( vector<Prob>::iterator mij = _messages.begin(); mij != _messages.end(); mij++ )
            mij->fill(1.0 / mij->size());
        _newmessages = _messages;
    }

    void TRW::calcNewMessage (size_t iI) {
        // calculate updated message I->i
        size_t i = grm().edge(iI).first;
        size_t I = grm().edge(iI).second;
        Var v_i = grm().var(i);

        VarSet v_I = grm().factor(I).vars();
        if( v_I.size() == 1 ) {
            _newmessages[iI] = grm().factor(I).p();
        } else if( v_I.size() == 2 ) {
            Var v_j = *((v_I / v_i).begin());
            size_t j = grm().findVar( v_j );

            Factor prod = grm().factor(I)^(1.0 / rho(i,j));

            Prob prod_j( v_j.states() );
            for( FactorGraph::nb_cit J = grm().nbV(j).begin(); J != grm().nbV(j).end(); J++ ) {
                VarSet v_J = grm().factor(*J).vars();
                if( v_J.size() == 1 )
                    prod_j *= message(j,*J);
                else if( v_J.size() == 2 ) {
                    Var v_k = *((v_J / v_j).begin());
                    size_t k = grm().findVar( v_k );
                    if( *J == I )
                        prod_j *= message(j,*J)^(rho(j,k) - 1.0);
                    else
                        prod_j *= message(j,*J)^rho(j,k);
                } else
                    DAI_THROW(NOT_IMPLEMENTED);
            }
            prod *= Factor( v_j, prod_j );

            _newmessages[iI] = prod.marginal(grm().var(i)).p();
        } else
            DAI_THROW(NOT_IMPLEMENTED);
    }


    double TRW::runTRW() {
        if( Props.verbose >= 1 )
            cout << "Starting " << identify() << ":inner_loop ...";
        if( Props.verbose >= 3)
           cout << endl; 

        init_messages();

        clock_t tic = toc();
        Diffs   diffs(grm().nrVars(), 1.0);
        
        vector<size_t> edge_seq;

        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back(beliefV(i));

        edge_seq.reserve( grm().nrEdges() );
        for( size_t e = 0; e < grm().nrEdges(); e++ )
            edge_seq.push_back( e );

        double logZ_prev = INFINITY;
        double logZ_cur  = real(logZ());

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        // and logZ difference is smaller than tolerance
        double diff = INFINITY;
        for( _iterations = 0; _iterations < Props.maxiter && diff > Props.tol; _iterations++ ) {
            if( Props.updates == UpdateType::PARALL ) {
                // Parallel updates 
                for( size_t t = 0; t < grm().nrEdges(); t++ )
                    calcNewMessage(t);

                for( size_t t = 0; t < grm().nrEdges(); t++ )
                    updateMessage( t );
            } else {
                // Sequential updates
                if( Props.updates == UpdateType::SEQRND )
                    random_shuffle( edge_seq.begin(), edge_seq.end() );
                
                for( size_t t = 0; t < grm().nrEdges(); t++ ) {
                    size_t k = edge_seq[t];
                    calcNewMessage( k );
                    updateMessage( k );
                }
            }

            // calculate new beliefs and compare with old ones
            for( size_t i = 0; i < grm().nrVars(); i++ ) {
                Factor nb( beliefV(i) );
                diffs.push( dist( nb, old_beliefs[i], Prob::DISTLINF ) );
                old_beliefs[i] = nb;
            }
            logZ_prev = logZ_cur;
            logZ_cur  = real(logZ());

            if( Props.verbose >= 3 )
                cout << "TRW::runTRW:  maxdiff " << diffs.max() << ", logZ_diff " << fabs(logZ_cur - logZ_prev) << " after " << _iterations+1 << " passes" << endl;

            diff = diffs.max();
            if( fabs(logZ_cur - logZ_prev) > diff )
                diff = fabs(logZ_cur - logZ_prev);
        }

        if( diff > _maxdiff )
            _maxdiff = diff;

        if( Props.verbose >= 1 ) {
            if( diff > Props.tol ) {
                if( Props.verbose == 1 )
                    cout << endl;
                    cout << "TRW::runTRW:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diff << endl;
            } else {
                if( Props.verbose >= 3 )
                    cout << "TRW::runTRW:  ";
                    cout << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diff;
    }


    double TRW::run() {
        if( Props.verbose >= 1 )
            cout << "Starting " << identify() << "..." << endl;

        init();

        inner_loop();

        double prev_logZ = real(logZ());
        double cur_logZ = prev_logZ;
        double alpha = 0.5;
        for( size_t iter = 0; iter < Props.maxouteriter; iter++ ) {
            double weight = calc_rho_descent();
            // weight = - <nabla f, \tilde rho_e_descent>

            if( Props.verbose >= 2 )
                cout << endl << "iter = " << iter << ", cur_logZ = " << cur_logZ << ", weight = " << weight << endl;
            if( std::isnan( weight ) ) {
                if( Props.verbose >= 1 )
                    cout << endl << "Reached NAN! Not going further...you'll have to use the current result" << endl;
                break;  // don't proceed, as this will lead to even more nan's
            }

            WeightedGraph<double> cur_rho = _rho_e;
            map<double,double> alpha_gain;
            bool ready = false;
            do {
                _rho_e = cur_rho;
                if( Props.verbose >= 3 )
                    cout << "checking alpha = " << alpha << "..." << endl;
                update_rho_e(alpha); // rho_e = (1-alpha) cur_rho + alpha rho_descent

                // calculate everything at rho_e
                inner_loop();

                cur_logZ = real(logZ());
                double gain = prev_logZ - cur_logZ;  // should be positive
                if( Props.verbose >= 3 )
                    cout << "  for alpha = " << alpha << ", gain = " << gain << " >=? alpha * weight = " << alpha * weight << ", gain / (alpha * weight) = " << gain / (alpha * weight) << endl;

                if( ready )
                    break;

                alpha_gain[alpha] = gain;

                map<double,double>::const_iterator best = alpha_gain.begin();       // alpha with highest gain
                map<double,double>::const_iterator nan = alpha_gain.end();          // lowest alpha with gain==NAN
                map<double,double>::const_iterator last = alpha_gain.begin();       // highest alpha with gain!=NAN
                for( map<double,double>::const_iterator pos = alpha_gain.begin(); pos != alpha_gain.end(); pos++ ) {
                    if( std::isnan(pos->second) ) {
                        if( nan == alpha_gain.end() )
                            nan = pos;
                    } else {
                        last = pos;
                        if( pos->second > best->second )
                            best = pos;
                    }
                }
                if( Verbose() >= 3 )
                    for( map<double,double>::const_iterator pos = alpha_gain.begin(); pos != alpha_gain.end(); pos++ )
                        cout << "      " << ((pos == best) ? "*" : " ") << pos->first << " " << pos->second << endl;
                if( best == alpha_gain.begin() )
                    alpha = best->first / 2.0;
                else if( best == last ) {
                    if( nan == alpha_gain.end() )
                        alpha = (1.0 + best->first) / 2.0;
                    else
                        alpha = (nan->first + best->first) / 2.0;
                } else {
                    // fit a parabola through three points around best
                    best--; double x1 = best->first; double y1 = best->second;
                    best++; double x2 = best->first; double y2 = best->second;
                    best++; double x3 = best->first; double y3 = best->second;
                    assert( y1 <= y2 );
                    assert( y3 <= y2 );
                    if( (x3 - x1) < Props.tol || !((y2 < y1) && (y2 < y3)) )
                        alpha = x2;
                    else { // this is unstable if (x3-x1) becomes too small...
                        if( Props.verbose >= 3 ) {
                            cout << "  fitting parabola through ";
                            cout.precision( 15 );
                            cout << "(x1,y1) = (" << x1 << "," << y1 << "), ";
                            cout << "(x2,y2) = (" << x2 << "," << y2 << "), ";
                            cout << "(x3,y3) = (" << x3 << "," << y3 << ")." << endl;
                        }
                        alpha = 0.5 * (x1*x1*(y3-y2) + x2*x2*(y1-y3) + x3*x3*(y2-y1)) / (x1*(y3-y2) + x2*(y1-y3) + x3*(y2-y1));
                    }
                    assert( 0.0 < alpha );
                    assert( alpha < 1.0 );
                    assert( x1 <= alpha );
                    if( !(alpha <= x3) ) {
                        cerr.precision( 15 );
                        cerr << "alpha = " << alpha << endl;
                        cerr << "(x1,y1) = (" << x1 << "," << y1 << "), ";
                        cerr << "(x2,y2) = (" << x2 << "," << y2 << "), ";
                        cerr << "(x3,y3) = (" << x3 << "," << y3 << ")." << endl;
                        if( (x3 - x1) < Props.tol || !((y2 < y1) && (y2 < y3)) )
                            cerr << "used alpha = x2" << endl;
                        else
                            cerr << "fitted parabola" << endl;
                    }
                    assert( alpha <= x3 );
                    ready = true;
                }
                if( alpha < Props.tol ) {
                    alpha = 0.0;    // stay where you are in this case
                    _rho_e = cur_rho;
                    // calculate everything at rho_e
                    inner_loop();
                    cur_logZ = real(logZ());
                }
            } while( alpha > Props.tol ); 
            assert( !std::isnan(cur_logZ) );
            if( Props.verbose >= 3 ) {
                cout << "Accepted alpha = " << alpha << ", this gives improvement of logZ of " << prev_logZ - cur_logZ << endl;
            }

            if( fabs( cur_logZ - prev_logZ ) < Props.outertol )  {
                if( Props.verbose >= 1 )
                    cout << "Reached tolerance! Ready!" << endl;
                break; // reached tolerance
            }
            prev_logZ = cur_logZ;
        }

        assert( !std::isnan( cur_logZ ) );
        assert( !std::isnan( prev_logZ ) );
        return( fabs( cur_logZ - prev_logZ ) );
    }
/*    double TRW::run_old() {
        if( Props.verbose >= 1 )
            cout << "Starting " << identify() << "..." << endl;

        init();

        inner_loop();

        double prev_logZ = real(logZ());
        double cur_logZ = prev_logZ;
//        double Armijo = 0.1;
        for( size_t iter = 0; iter < Props.maxouteriter; iter++ ) {
            double weight = calc_rho_descent();
            if( Props.verbose >= 2 )
                cout << endl << "iter = " << iter << ", cur_logZ = " << cur_logZ << ", weight = " << weight << endl;
            // weight = - <nabla f, \tilde rho_e_descent>

            WeightedGraph<double> cur_rho = _rho_e;
//            double alpha = 0.999;     first pass is rather slow, maybe this step is too large
            double alpha = 0.5;
            double prev_gain = -INFINITY;
            do {
                _rho_e = cur_rho;
                if( Props.verbose >= 3 )
                    cout << "checking alpha = " << alpha << "..." << endl;
                update_rho_e(alpha); // rho_e = (1-alpha) cur_rho + alpha rho_descent

//                if( std::isnan( prev_gain ) )
//                    init_messages();

                // calculate everything at rho_e
                inner_loop();

                cur_logZ = real(logZ());
                double gain = prev_logZ - cur_logZ;  // should be positive
                if( Props.verbose >= 3 )
                    cout << "  for alpha = " << alpha << ", gain = " << gain << " >=? alpha * weight = " << alpha * weight << ", gain / (alpha * weight) = " << gain / (alpha * weight) << endl;

                // Joris' rule:
                if( gain > 0 && (gain < prev_gain) )
                    break;  // we should not make alpha any smaller, since the gain will only deteriorate

                // Armijo's rule for sigma = 0.1:
                // does not work very well...or maybe I just don't understand how you should choose sigma
//                if( (gain / (alpha * weight)) > Armijo )
//                     break;

                alpha /= 2.0;
                prev_gain = gain;
            } while( alpha > Props.tol ); 
            if( Props.verbose >= 3 ) {
                cout << "accepted alpha = " << alpha << ", this gives improvement of logZ of " << prev_logZ - cur_logZ << endl;
            }
            assert( alpha > Props.tol );  // this should not happen, hopefully!

            if( fabs( cur_logZ - prev_logZ ) < Props.outertol )  {
                if( Props.verbose >= 1 )
                    cout << "Reached tolerance! Ready!" << endl;
                break; // reached tolerance
            }
            prev_logZ = cur_logZ;
        }

        return( fabs( cur_logZ - prev_logZ ) );
    }
*/

    double TRW::inner_loop() {
        size_t old_verbose = Props.verbose;
        double old_damping = Props.damping;

        Props.verbose = (Props.verbose >= 2) ? Props.verbose - 2 : 0;
        double diff = 0.0;
        do {
            init_messages();
            diff = runTRW();
            if( diff > Props.tol ) {
                Props.damping = (2.0 * Props.damping + 1.0) / 3.0;
                if( old_verbose >= 2 )
                    cout << "Need damping, trying " << Props.damping << ", hopefully it helps..." << endl;
            }
        } while( diff > Props.tol );

        Props.verbose = old_verbose;
        Props.damping = old_damping;

        return diff;
    }


    Factor TRW::beliefV( size_t i ) const {
        Var v_i = grm().var(i);

        Prob prod( v_i.states() ); 
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) {
            VarSet v_I = grm().factor(*I).vars();

            if( v_I.size() == 1 )
                prod *= newMessage(i,*I);
            else if( v_I.size() == 2 ) {
                Var v_j = *((v_I / v_i).begin());
                size_t j = grm().findVar( v_j );
                prod *= newMessage(i,*I)^rho(i,j);
            } else
                DAI_THROW(NOT_IMPLEMENTED);
        }

        prod.normalize();
        return( Factor( grm().var(i), prod ) );
    }


    Factor TRW::belief (const Var &n) const {
        return( beliefV( grm().findVar( n ) ) );
    }


    vector<Factor> TRW::beliefs() const {
        vector<Factor> result;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            result.push_back( beliefV(i) );
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            result.push_back( beliefF(I) );
        return result;
    }


    Factor TRW::belief( const VarSet &ns ) const {
        if( ns.size() == 1 )
            return belief( *(ns.begin()) );
        else {
            size_t I;
            for( I = 0; I < grm().nrFactors(); I++ )
                if( grm().factor(I).vars() >> ns )
                    break;
            assert( I != grm().nrFactors() );
            return beliefF(I).marginal(ns);
        }
    }


    Factor TRW::beliefF (size_t I) const {
        VarSet v_I = grm().factor(I).vars();
        
        Factor prod;
        if( v_I.size() == 1 )
            ;
        else if( v_I.size() == 2 ) {
            Var v_i = *(v_I.begin());
            Var v_j = *((v_I / v_i).begin());
            size_t i = grm().findVar(v_i);
            size_t j = grm().findVar(v_j);
            prod = grm().factor(I)^(1.0 / rho(i,j));
        } else
            DAI_THROW(NOT_IMPLEMENTED);

        for( FactorGraph::nb_cit i = grm().nbF(I).begin(); i != grm().nbF(I).end(); i++ ) {
            Var v_i = grm().var( *i );
            Prob prod_i( v_i.states() );

            for( FactorGraph::nb_cit J = grm().nbV(*i).begin(); J != grm().nbV(*i).end(); J++ ) {
                VarSet v_J = grm().factor(*J).vars();
                if( v_J.size() == 1 ) {
                    prod_i *= newMessage(*i,*J);
                } else if( v_J.size() == 2 ) {
                    Var v_k = *((v_J / v_i).begin());
                    size_t k = grm().findVar(v_k);
                    if( *J != I )
                        prod_i *= newMessage(*i,*J)^rho(*i,k);
                    else
                        prod_i *= newMessage(*i,*J)^(rho(*i,k) - 1.0);
                } else
                    DAI_THROW(NOT_IMPLEMENTED);
            }

            prod *= Factor( v_i, prod_i );
        }

        return prod.normalized();
    }


    Complex TRW::logZ() const {
/*        Complex sum = 0.0;
        for(size_t i = 0; i < grm().nrVars(); i++ )
            sum += Complex(1.0 - grm().nbV(i).size()) * beliefV(i).entropy();
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            sum -= KL_dist( beliefF(I), grm().factor(I) );
        return sum;*/

        Real sum = 0.0;
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            sum += (beliefF(I) * grm().factor(I).log()).totalSum();
        for( size_t i = 0; i < grm().nrVars(); i++ ) {
            Var v_i = grm().var(i);
            sum += real(beliefV(i).entropy());
            for( size_t j = i+1; j < grm().nrVars(); j++ ) {
                Var v_j = grm().var(j);
                if( grm().delta(v_i) && v_j )
                    sum -= rho(i,j) * MutualInfo( belief( v_i | v_j ) );
            }
        }
        return sum;
    }


    string TRW::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void TRW::init( const VarSet &ns ) {
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) {
            size_t ni = grm().findVar( *n );
            for( FactorGraph::nb_cit I = grm().nbV(ni).begin(); I != grm().nbV(ni).end(); I++ )
                message(ni,*I).fill( 1.0 );
        }
    }


}
