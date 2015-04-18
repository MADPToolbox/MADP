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
#include "bp.h"
#include "diffs.h"
#include "util.h"
#include "properties.h"
#include "exceptions.h"


namespace libDAI {


    using namespace std;


    const char *BP::Name = "BP";


    bool BP::initProps() {
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

        return true;
    }


    BP::BP(const FactorGraph & fg, const Properties &opts) : DAIAlgFG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), _indices(), _messages(), _newmessages() {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        // clear messages
        _messages.clear();
        _messages.reserve(grm().nrEdges());

        // clear indices
        _indices.clear();
        _indices.reserve(grm().nrEdges());

        // create messages and indices
        for( size_t iI = 0; iI < grm().nrEdges(); iI++ ) {
            size_t i = grm().edge(iI).first;
            size_t I = grm().edge(iI).second;

            _messages.push_back( Prob( grm().var(i).states() ) );

            vector<size_t> ind( grm().factor(I).stateSpace(), 0 );
            Index indi (grm().var(i), grm().factor(I).vars() );
            for( size_t j = 0; indi >= 0; ++indi,++j )
                ind[j] = indi; 
            _indices.push_back( ind );
        }

        // create new_messages
        _newmessages = _messages;
    }


    void BP::init() { //messages (one for each edge?) are initialized with uniform probabilities
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        for( vector<Prob>::iterator mij = _messages.begin(); mij != _messages.end(); mij++ )
            mij->fill(1.0 / mij->size());
        _newmessages = _messages;
    }


    void BP::calcNewMessage (size_t iI) { //FRANS: iI is the index of the edge in the factorgraph...
        // calculate updated message I->i
        size_t i = grm().edge(iI).first;    //FRANS: here we get the index of the variable (? 'first' type) corresponding to edge iI
        size_t I = grm().edge(iI).second;   //FRANS: here we get the index of the factor (? the 'second' type) corresponding to edge iI
        ///FRANS: indeed 1st=var, 2nd=Factor, see "typedef BipartiteGraph<Var,Factor> BipFacGraph;" in factorgraph.h, 38

    /*  UNOPTIMIZED (SIMPLE TO READ, BUT SLOW) VERSION

        Factor prod( grm().factor( I ) );
        for( FactorGraph::nb_cit j = grm().nbF(I).begin(); j != grm().nbF(I).end(); j++ )   //FRANS: nbF(I)  returns a list to neighbors of Factor I (of the graph.model) (implementation-wise this list is a vector<size_t> )
            if( *j != i ) {     // for all j in I \ i                                       //FRANS: FactorGraph::nb_cit is a typedef provides an iterator of the list of neighbors
                for( FactorGraph::nb_cit J = grm().nbV(*j).begin(); J != grm().nbV(*j).end(); J++ ) 
                    if( *J != I ) {     // for all J in nb(j) \ I 
                        prod *= Factor( *j, message(*j,*J) );
        Factor marg = prod.marginal(grm().var(i));
    */
        
        Prob prod( grm().factor(I).p() );

        // Calculate product of incoming messages and factor I
        for( FactorGraph::nb_cit j = grm().nbF(I).begin(); j != grm().nbF(I).end(); j++ )
            if( *j != i ) {     // for all j in I \ i
                // ind is the precalculated Index(j,I) i.e. to x_I == k corresponds x_j == ind[k]
                _ind_t* ind = &(index(*j,I));

                // prod_j will be the product of messages coming into j
                Prob prod_j( grm().var(*j).states() ); 
                for( FactorGraph::nb_cit J = grm().nbV(*j).begin(); J != grm().nbV(*j).end(); J++ ) 
                    if( *J != I )   // for all J in nb(j) \ I 
                        prod_j *= message(*j,*J);

                // multiply prod with prod_j
                for( size_t r = 0; r < prod.size(); r++ )
                    prod[r] *= prod_j[(*ind)[r]];
            }

        // Marginalize onto i
        Prob marg( grm().var(i).states(), 0.0 );
        // ind is the precalculated Index(i,I) i.e. to x_I == k corresponds x_i == ind[k]
        _ind_t* ind = &(index(i,I));
        for( size_t r = 0; r < prod.size(); r++ )
            marg[(*ind)[r]] += prod[r];
        marg.normalize( grm().normType() );
        
        // Store result
        _newmessages[iI] = marg;
    }


    // BP::run does not check for NANs for performance reasons
    // Somehow NaNs do not often occur in BP...
    double BP::run() {
        if( Props.verbose >= 1 )
            cout << "Starting " << identify() << "...";
        if( Props.verbose >= 3)
           cout << endl; 

        clock_t tic = toc();
        Diffs diffs(grm().nrVars(), 1.0);
        
        vector<size_t> edge_seq; //what does this contain? an ordering (sequence) of edge(indice)s ?
        vector<double> residuals;

        vector<Factor> old_beliefs;
        old_beliefs.reserve( grm().nrVars() );
        for( size_t i = 0; i < grm().nrVars(); i++ )
            old_beliefs.push_back(beliefV(i));

        if( Props.updates == UpdateType::SEQMAX ) {
            // do the first pass
            for(size_t iI = 0; iI < grm().nrEdges(); iI++ ) 
                calcNewMessage(iI);

            // calculate initial residuals
            residuals.reserve(grm().nrEdges());
            for( size_t iI = 0; iI < grm().nrEdges(); iI++ )
                residuals.push_back( dist( _newmessages[iI], _messages[iI], Prob::DISTLINF ) );
        } else {
            edge_seq.reserve( grm().nrEdges() );
            for( size_t e = 0; e < grm().nrEdges(); e++ )
                edge_seq.push_back( e );
        }

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) {
            if( Props.updates == UpdateType::SEQMAX ) {
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
                for( size_t t = 0; t < grm().nrEdges(); t++ )
                    calcNewMessage(t);

                for( size_t t = 0; t < grm().nrEdges(); t++ )
                    updateMessage( t );
            } else {
                // Sequential updates
                if( Props.updates == UpdateType::SEQRND )
                    random_shuffle( edge_seq.begin(), edge_seq.end() );
                
                for( size_t t = 0; t < grm().nrEdges(); t++ ) {
                    size_t k = edge_seq[t]; //FRANS: k is what?
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

            if( Props.verbose >= 3 )
                cout << "BP::run:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;
        }

        if( diffs.max() > _maxdiff )
            _maxdiff = diffs.max();

        if( Props.verbose >= 1 ) {
            if( diffs.max() > Props.tol ) {
                if( Props.verbose == 1 )
                    cout << endl;
                    cout << "BP::run:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            } else {
                if( Props.verbose >= 3 )
                    cout << "BP::run:  ";
                    cout << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
            }
        }

        return diffs.max();
    }


    Factor BP::beliefV( size_t i ) const {
        Prob prod( grm().var(i).states() ); 
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) 
            prod *= newMessage(i,*I);

        prod.normalize();
        return( Factor( grm().var(i), prod ) );
    }


    Factor BP::belief (const Var &n) const {
        return( beliefV( grm().findVar( n ) ) );
    }


    vector<Factor> BP::beliefs() const {
        vector<Factor> result;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            result.push_back( beliefV(i) );
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            result.push_back( beliefF(I) );
        return result;
    }


    Factor BP::belief( const VarSet &ns ) const {
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


    Factor BP::beliefF (size_t I) const {
        Prob prod( grm().factor(I).p() );

        for( FactorGraph::nb_cit j = grm().nbF(I).begin(); j != grm().nbF(I).end(); j++ ) {
            // ind is the precalculated Index(j,I) i.e. to x_I == k corresponds x_j == ind[k]
            const _ind_t *ind = &(index(*j, I));

            // prod_j will be the product of messages coming into j
            Prob prod_j( grm().var(*j).states() ); 
            for( FactorGraph::nb_cit J = grm().nbV(*j).begin(); J != grm().nbV(*j).end(); J++ ) 
                if( *J != I )   // for all J in nb(j) \ I 
                    prod_j *= newMessage(*j,*J);

            // multiply prod with prod_j
            for( size_t r = 0; r < prod.size(); r++ )
                prod[r] *= prod_j[(*ind)[r]];
        }

        Factor result( grm().factor(I).vars(), prod );
        result.normalize();
        
        return( result );

    /*  UNOPTIMIZED VERSION
     
        Factor prod( grm().factor(I) );
        for( FactorGraph::nb_cit i = grm().nbF(I).begin(); i != grm().nbF(I).end(); i++ ) {
            for( FactorGraph::nb_cit J = grm().nbV(*i).begin(); J != grm().nbV(*i).end(); J++ )
                if( *J != I )
                    prod *= Factor( grm().var(*i), newMessage(*i,*J)) );
        }
        return prod.normalized();*/
    }


    Complex BP::logZ() const {
        Complex sum = 0.0;
        for(size_t i = 0; i < grm().nrVars(); i++ )
            sum += Complex(1.0 - grm().nbV(i).size()) * beliefV(i).entropy();
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            sum -= KL_dist( beliefF(I), grm().factor(I) );
        return sum;
    }


    string BP::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void BP::init( const VarSet &ns ) {
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) {
            size_t ni = grm().findVar( *n );
            for( FactorGraph::nb_cit I = grm().nbV(ni).begin(); I != grm().nbV(ni).end(); I++ )
                message(ni,*I).fill( 1.0 );
        }
    }


}
