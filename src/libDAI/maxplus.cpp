/*  Copyright (C) 2008  Frans Oliehoek  [F.A.Oliehoek@uva.nl], Joris Mooij  [joris at jorismooij dot nl]
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

#include <stdlib.h>

//timing results need to following
#include <sys/time.h>
#include <time.h>

#include "maxplus.h"
#include "diffs.h"
#include "util.h"
#include "properties.h"
#include "exceptions.h"
#include "MADP_util.h"


//verboseness levels:
//0 - silent
//1 - top level info
//2
//3
//4 - 
//5 - debug inserting ne configs
//6 - print messages debug stuff


namespace libDAI {
    using namespace std;
    using namespace libDAI::MADP_util;


    const char *MaxPlus::Name = "MaxPlus";


    bool MaxPlus::initProps() {
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


    MaxPlus::MaxPlus(const FactorGraph & fg, const Properties &opts, size_t k ) 
        : 
            DAIAlgFG(fg, opts), Props(), _maxdiff(0.0), _iterations(0UL), 
            _indices(), _messages(), _newmessages(),
            _g(&fg) , bestConfiguration( fg.nrVars() ), 
            _k(k),
            _k_th_Val(-DBL_MAX),
            writeAnyTimeResults(false), results_f(NULL), timings_f(NULL)
    {
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


    void MaxPlus::init() { //messages (one for each edge?) are initialized with uniform probabilities
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        for( vector<Prob>::iterator mij = _messages.begin(); mij != _messages.end(); mij++ )
            //mij->fill(1.0 / mij->size());
            //mij->fill(0.0 ); //FRANS Max-plus initialized on 0.0
            mij->fill( (double) rand() * 50 / RAND_MAX ); //FRANS Max-plus initialized on 0.0
        _newmessages = _messages;
        if( Props.verbose >= 1)
            cout << "Max-plus initialized at verbose="<<Props.verbose<<endl;
    }


    void MaxPlus::calcNewMessage (size_t iI) 
    { 
        const FactorGraph& g = grm();
        //FRANS: iI is the index of the edge in the factorgraph...
        //this function calculates the updated message I->i
        size_t i = g.edge(iI).first;    //FRANS: here we get the index of the variable (? 'first' type) corresponding to edge iI
        size_t I = g.edge(iI).second;   //FRANS: here we get the index of the factor (? the 'second' type) corresponding to edge iI
        ///FRANS: indeed 1st=var, 2nd=Factor, see "typedef BipartiteGraph<Var,Factor> BipFacGraph;" in factorgraph.h, 38
        if( Props.verbose >= 6)
            cout << "new Mess F"<< I <<" -> v"<< i<<endl;

        Prob sum( g.factor(I).p() );

        //if( Props.verbose >= 8)
        //cout<<"\tInitialized sum = " << sum << endl;

        //FRANS: nbF(I)  returns a list to neighbors of Factor I (of the 
        // graph.model) (implementation-wise this list is a vector<size_t> )
        //FRANS: FactorGraph::nb_cit is a typedef provides an iterator of 
        // the list of neighbors
        //Prob & m; //can we make this a reference?
        for( FactorGraph::nb_cit j = g.nbF(I).begin(); j != g.nbF(I).end(); j++ )   
            if( *j != i ) {     // for all j in I \ i                                       
                //if( Props.verbose >= 5)
                    //cout << "\t\tneighbor variable v"<<*j<<" of factor F"<<I<<endl;

                // ind is the precalculated Index(j,I) i.e. to x_I == k corresponds x_j == ind[k]
                _ind_t* ind = &(index(*j,I));
                // sum_j will be the sum of messages coming into j
                Prob sum_j( g.var(*j).states(), 0.0 ); 
                //if( Props.verbose >= 8)
                    //cout << "\t\tInitialized sum_j = "<< sum_j << "(j=" << *j << ")" << endl;

                for( FactorGraph::nb_cit J = g.nbV(*j).begin(); J != g.nbV(*j).end(); J++ ) 
                    if( *J != I ) {     
                        // for all J in nb(j) \ I 
                        //if( Props.verbose >= 5)
                            //cout << "\t\tneighbor factor F"<<*J<<" of variable v"<<*j<<endl;
                        // message(*j,*J) is the (previous) message received by j from J.
                        const Prob & m =  message(*j,*J);
                        //if( Props.verbose >= 5)
                            //cout << "\t\tPrev. message from  F" <<*J<< " -> v" <<*j<< " = " <<m<<endl;
                        sum_j += m;
                        //if( Props.verbose >= 7)
                            //cout << "\t\tsum_j = "<< sum_j <<endl;
                    } 

                // add sum_j to sum
                //if( Props.verbose >= 6)
                    //cout << "\tadding sum_j = "<< sum_j << " to sum = " << sum << endl;
                    
                //we need to add sum_j[k] to all entries sum[r] in sum that are consistent with sum_j[k],
                //i.e., all entries where variable j has state k.
                for( size_t r = 0; r < sum.size(); r++ )
                {
                    //size_t sum_j_index = (*ind)[r];
                    //if( Props.verbose >= 9)
                        //cout << "\t\tr="<<r<<", sum_j_index="<<sum_j_index<<endl;
                    sum[r] += sum_j[(*ind)[r]];
                }
        
            }
     
        //if( Props.verbose >= 6)
            //cout<<"\tsum = " << sum << endl;
        //old slow code....
        //Factor sum_factor ( g.factor( I ).vars(), sum);//create a factor...
        ////...which allows use to easily take the max:
        //Factor max_f = sum_factor.partMax(g.var(i)); //here we take the Max

        //make all changes directly to the new messages:
        Prob & newm = _newmessages[iI];
        //initialize to -infty
        for( size_t i = 0; i < newm.size(); i++)
            newm[i] = -INFINITY;

        //now we compute the max inline (it's an inline version of factor::partMax )
        const VarSet & ns = g.var(i);
        const VarSet & vs = g.factor( I ).vars();
        //Prob res( g.var(i).states(), -INFINITY );
        //Index i_res( ns, vs );
        //for( size_t i = 0; i < sum.size(); i++, ++i_res )
            //res[i_res] = std::max(res[i_res], sum[i]);

        Index i_newm( ns, vs );
        for( size_t i = 0; i < sum.size(); i++, ++i_newm )
            newm[i_newm] = std::max(newm[i_newm], sum[i]);

        //Prob max = max_f.p(); //here we get the desired vector (Prob) repres.
        //Prob & max = res; 
        //if( Props.verbose >= 7)
            //cout << "\t\tsum_factor = "<<sum_factor<<", max = "<<max<<endl;

        // substract average of messages to stop growing
        //size_t s = max.size();
        //double avg = max.totalSum() / s;
        //Prob newm = max - Prob(s, avg); // compute the new normalized message

        double avg = newm.totalSum() / newm.size();
        for( size_t i = 0; i < newm.size(); i++)
            newm[i] -= avg;
        

        // Store result
        //_newmessages[iI] = newm;

        if( Props.verbose >= 6)
            cout << "\tnew Mess F"<< I <<" -> v"<< i<<" = "<<_newmessages[iI]<<endl;
    }

#define DEBUG_DAMP 0    
    void MaxPlus::updateMessage(size_t iI) {
        if( Props.damping <= 0.00001 )
        {
#if DEBUG_DAMP
            cout << " MaxPlus::updateMessage - No damping" << endl;
#endif            
            _messages[iI] = _newmessages[iI];
        }
        else
        {
            //this does not work correctly for arbitrary numbers
            //_messages[iI] = (_messages[iI] ^ Props.damping) * (_newmessages[iI] ^ (1.0 - Props.damping));
            //rather use regular pointwise multiplication
#if DEBUG_DAMP
            cout << " MaxPlus::updateMessage - using damping, damp factor=" << 
                Props.damping<<"..."<<endl;
            cout << "old messages     " << _messages[iI] << endl;
            cout << "new messages     " << _newmessages[iI] << endl;            
#endif            
            //inlined this code to speed it up (no need to create new 
            //vectors etc.)
            //_messages[iI] = (_messages[iI] * Props.damping) + (_newmessages[iI] * (1.0 - Props.damping));
            Prob & this_message = _messages[iI];
            Prob & new_message = _newmessages[iI];
            double d = Props.damping;
            double nd = 1.0 - d;
            for(size_t i=0; i < this_message.size(); i++)
            {
                this_message[i] *= d;
                this_message[i] += nd * new_message[i];
            }
                 

#if DEBUG_DAMP
            cout << "updated messages " << _messages[iI] << endl;            
#endif            
        }
    }

    // MaxPlus::run does not check for NANs for performance reasons
    // Somehow NaNs do not often occur in MaxPlus...
    double MaxPlus::run() {
        const FactorGraph& g = grm();
        bestValue = -INFINITY;
        if( Props.verbose >= 2 )
            cout << "Starting " << identify() << "..."<<endl;
        if( Props.verbose >= 3)
           cout << endl; 

        struct timeval start_time, cur_time;
        if(gettimeofday(&start_time, NULL) != 0)
            throw "Error with gettimeofday";

        clock_t tic = toc();
        Diffs diffs(g.nrVars(), 1.0);
        
        vector<size_t> edge_seq; //what does this contain? an ordering (sequence) of edge(indice)s ?
        vector<double> residuals;


        //vector<Factor> old_beliefs;
        vector<Prob> old_beliefs;
        old_beliefs.reserve( g.nrVars() );
        for( size_t i = 0; i < g.nrVars(); i++ )
            //old_beliefs.push_back(beliefV(i));
            //FRANS: we don't want a belief in here but simple a vector with all 0's
            //old_beliefs.push_back(  Factor( g.var(i), 0.0 )  );
            old_beliefs.push_back(  Prob( g.var(i).states(), 0.0 )  );

        if( Props.updates == UpdateType::SEQMAX ) {
            // do the first pass
            for(size_t iI = 0; iI < g.nrEdges(); iI++ ) 
                calcNewMessage(iI);

            // calculate initial residuals
            residuals.reserve(g.nrEdges());
            for( size_t iI = 0; iI < g.nrEdges(); iI++ )
                residuals.push_back( dist( _newmessages[iI], _messages[iI], Prob::DISTLINF ) );
        } else {
            edge_seq.reserve( g.nrEdges() );
            for( size_t e = 0; e < g.nrEdges(); e++ )
                edge_seq.push_back( e );
        }

        // do several passes over the network until maximum number of iterations has
        // been reached or until the maximum belief difference is smaller than tolerance
        for( _iterations = 0; _iterations < Props.maxiter && diffs.max() > Props.tol; _iterations++ ) 
        {
            if( Props.verbose >= 6)
            {
               cout << "Start iteration "<<_iterations<<endl ; 
               cout << "old messages:"<< endl << _messages << endl;
            }

            // ---------First update the messages------------
            if( Props.updates == UpdateType::SEQMAX ) {
                // Residuals-MaxPlus by Koller et al.
                for( size_t t = 0; t < g.nrEdges(); t++ ) {
                    // update the message with the largest residual
                    size_t iI = max_element(residuals.begin(), residuals.end()) - residuals.begin();
                    updateMessage( iI );
                    residuals[iI] = 0;

                    // I->i has been updated, which means that residuals for all
                    // J->j with J in nb[i]\I and j in nb[J]\i have to be updated
                    size_t i = g.edge(iI).first;
                    size_t I = g.edge(iI).second;
                    for( FactorGraph::nb_cit J = g.nbV(i).begin(); J != g.nbV(i).end(); J++ ) 
                        if( *J != I )
                            for( FactorGraph::nb_cit j = g.nbF(*J).begin(); j != g.nbF(*J).end(); j++ )
                                if( *j != i ) {
                                    size_t jJ = g.edge(*j,*J);
                                    calcNewMessage(jJ);
                                    residuals[jJ] = dist( _newmessages[jJ], _messages[jJ], Prob::DISTLINF );
                                }
                }
            } else if( Props.updates == UpdateType::PARALL ) {
                // Parallel updates 
                for( size_t t = 0; t < g.nrEdges(); t++ )
                    calcNewMessage(t);

                for( size_t t = 0; t < g.nrEdges(); t++ )
                    updateMessage( t );
            } else {
                // Sequential updates
                if( Props.updates == UpdateType::SEQRND )
                    random_shuffle( edge_seq.begin(), edge_seq.end() );
                
                for( size_t t = 0; t < g.nrEdges(); t++ ) {
                    size_t k = edge_seq[t]; //FRANS: k is what?
                    calcNewMessage( k );
                    updateMessage( k );
                }
            }
            
            // ---------End updating the messages------------


            // compute, for each variable, the value that maximizes the sum of the factors it 
            // participates in.
            // I.e., if v0 is var with 3 values ('states') a,b,c and participates in F0 and F1, 
            // and it received the following messages
            //      Mess F0 -> v0 = [0 6 0 ]
            //      Mess F1 -> v0 = [3 0 0 ]
            // then it should take value b, corresponding to the maximizing index in
            // the vector [3 6 0], which we call the 
            //  `estimated contribution vector' ECV

            //max_indices stores the indices of the maximizing values ('states') for each variable
            vector<size_t> max_indices( g.nrVars() );
            for( size_t i = 0; i < g.nrVars(); i++ )
            {
                /* need not use Factor class here...
                Factor ECV( g.var( i ), 0.0);
                Var i_th_var = g.var(i);
                VarSet temp_vs (i_th_var);
                //loop over factors I that are neighbors of var i
                for( FactorGraph::nb_cit I = g.nbV(i).begin(); I != g.nbV(i).end(); I++ )
                    ECV += Factor( temp_vs, message(i,*I) );
                */
                Prob ECV( g.var(i).states() , 0.0);
                //loop over factors I that are neighbors of var i
                for( FactorGraph::nb_cit I = g.nbV(i).begin(); I != g.nbV(i).end(); I++ )
                    ECV += message(i,*I);

                //find the maximum index (i.e., the value the variable should take according to ECV)
                Real Z = -INFINITY;
                size_t max_val_i = 0;
                for( size_t val_i = 0; val_i < ECV.size(); val_i++ ) {
                    if( ECV[val_i] > Z )
                    {
                        Z = ECV[val_i];
                        max_val_i = val_i;
                    }
                }
                max_indices[i] = max_val_i;
#if 0                
                if( Props.verbose >= 3)
                {
                    cout << "ECV for var "<<i<<" = "<< ECV ;
                    if( Props.verbose >= 4)
                        cout << "->best value of var "<<i<<" is " << max_val_i << " (with expected sum "<<Z<<")";
                    cout << endl;
                }
#endif                 

                //compute distance to last ECV
                Real d =  dist( ECV, old_beliefs[i], Prob::DISTLINF );
                diffs.push( d );
#if 0                
                if( Props.verbose >= 4 ) {
                    if( Props.verbose >= 6 )
                        cout << "\t\t\told_beliefs[i]"<< old_beliefs[i] <<endl << "\t\t\tdist d="<<d<<endl;
                    cout << "\t\tdiffs = "<<diffs<<endl;
                }
#endif                 
                old_beliefs[i] = ECV;
            }
            if( Props.verbose >= 3 )
                cout << "MaxPlus::run:  maxdiff " << diffs.max() << " after " << _iterations+1 << " passes" << endl;

            //compute the value of the currently chosen config
            //
            const std::vector<Factor> & factor_vec = g.factors();
            std::vector<Factor>::const_iterator it = factor_vec.begin();
            std::vector<Factor>::const_iterator last = factor_vec.end();

            double valuesum = 0.0;
            while(it != last) //loop over all factors to compute value
            {
                const Factor & f = *it;
                const VarSet & vs = f.vars();

                //we need to compute the index into f.p() that corresponds to 
                //the (state) configuration as defined by max_indices.
                //
                //suppose the state configuration is <2 1 3 5>, 
                //when f includes vars 1 2 and 4, the relevant configuration is <2 1 5>
                //suppose that the number of states of these relevant vars is <3 4 6>,
                //then the index should be computed as
                //2 + 1 * (3) + 5 * (3*4) = 2 + 3 + 60 = 65
                //
                //(remember the index with the lowest index is the least significant in libDAI)

                //first we construct the vector of 'relevant' indices (those for vs)
                vector<size_t> relevant_state_indices(vs.size());                
                int i=0;
                for(VarSet::const_iterator cit = vs.begin(); cit != vs.end(); cit++)
                { 
                    //the index ('label') of the next of the variable of vs:
                    long lab =  (*cit).label();                    
                    // the (index of the) value(state) that variable
                    //takes in the (estimated) maximizing configuration:
                    //!!! if it crashes here: is the FG connected 'enough' ?! (because it will give problems of not!)
                    //(i.e., if not all variables are included in a factor, things mess up here...)
                    size_t val = max_indices.at(lab);
                    //relevant_state_indices.push_back( val );
                    relevant_state_indices[i] = val;
                    i++;
                }
                //now compute the 'joint factor' index
                size_t facIndex = Factor::IndividualToJointFactorIndex(vs, relevant_state_indices);
                valuesum += f[facIndex];
                it++;
            }
            if( Props.verbose >= 3)
                cout << "current estimated maximizing configuration attains: " << 
                    valuesum << 
                    " (the k=" <<_k<< "-th Value is " << _k_th_Val << ")" << endl;

            if(valuesum > _k_th_Val + V_EPS)
            {
                if( Props.verbose >= 4)
                    cout << "\tchecking to add this configuration to solutions"<<endl;
                list< MADP_util::valConf >::iterator it = _valConfs.begin();
                //boolean that indicates whether we should store max_indices
                bool insert_element = true;
                int pos = 0;
                while(it != _valConfs.end() )
                {
                    valConf& vc = *it;
                    double vcval = vc.first;
                    
                    if( Props.verbose >= 5)
                        cout << "\tchecking to see if it should be inserted before the entry at position "<< pos << " (with value "<<vcval<<")..."<<endl;

                    //boolean that will indicate whether the new found policy needs
                    //to be inserted before vc
                    bool insert_before_vc = false;
                    if( valuesum + V_EPS < vcval)
                    {
                        insert_before_vc = true;
                        if( Props.verbose >= 5)
                            cout << "\tYes!  valuesum("<<valuesum<<") + V_EPS("<<
                                V_EPS<<") < vcval("<<vcval<<" )"<<endl;
                    }
                        
                    if( EqualValue(vcval, valuesum) )
                    {
                        //insert before vc 
                        insert_before_vc = true;
                        if( Props.verbose >= 5)
                            cout << "\tYes!  valuesum("<<valuesum<<") + V_EPS("<<
                                V_EPS<<") == vcval("<<vcval<<" )"<<endl;

                        //but make sure the condigurations are different!
                        vector<size_t>& vcconf=vc.second;
                        bool identical = true;
                        for( size_t i=0; i < vcconf.size(); i++)
                        {
                            if(max_indices.at(i) != vcconf[i] )
                            {
                                identical = false;
                                break;
                            }
                        }
                        if(identical)
                        {
                            if( Props.verbose >= 4)
                                cout <<"\tnewly found conf identical to one already stored..."<<endl;
                            insert_element = false;
                        }
                    }

                    if(insert_before_vc)
                        break;

                    it++;
                    pos++;
                }
                //it now points to the element before which max_indices should be
                //inserted
                if(it == _valConfs.begin() && _valConfs.size() == _k )
                {
                    if( Props.verbose >= 4)
                        cout <<"\tDon't bother value ("<< valuesum <<
                            ") not high enough..."<< endl;
                    //don't bother, since the new one become the k+1-th element
                    insert_element = false;
                }
                if(insert_element)
                {
                    //to avoid uneceesary copying, we insert an 'empty' valconf
                    vector<size_t> v_empty;
                    valConf empty_vc = make_pair(valuesum, v_empty );
                    list< valConf >::iterator new_el_it = _valConfs.insert(it, empty_vc);
                    valConf& new_vc = (*new_el_it);
                    if( Props.verbose >= 4)
                        cout <<"\tvalue of inserted valconf = "<< new_vc.first<<endl;
                    new_vc.second = max_indices;
                    //check to see if we need to delete the last element
                    if(_valConfs.size() > _k)
                        _valConfs.pop_front();
                    if(_valConfs.size() == _k)
                        _k_th_Val = _valConfs.front().first;
                }
                else
                    if( Props.verbose >= 4)
                        cout <<"\t*NOT* inserting the newly found configuration"<< endl;

            }
            if(valuesum > bestValue)
            {
                bestValue = valuesum;
                bestConfiguration = max_indices;
                if(gettimeofday(&cur_time, NULL) != 0)
                    throw "Error with gettimeofday";

                time_t delta_sec = cur_time.tv_sec - start_time.tv_sec;
                suseconds_t delta_usec = cur_time.tv_usec - start_time.tv_usec;
                double delta = 1000000.0 * delta_sec + delta_usec; //in microsecond

                if(writeAnyTimeResults){
                    (*results_f) << bestValue << "\t";
                    (*timings_f) << delta << "\t";
                } 

                if( Props.verbose >= 2)
                    cout << "new best configuration found, value = "<< bestValue << endl;
            }
            if( Props.verbose >= 6)
            {
                cout << "new messages:"<< endl << _messages << endl;
                cout << "ending iteration "<<_iterations<<endl<<endl ; 
            }
        } // <- end of iteration

        //end the line in the results file
        if(writeAnyTimeResults){
            (*results_f) << endl;
            (*timings_f) << endl;
        } 
        
        if( diffs.max() > _maxdiff ) //store the max difference for later inspection.
            _maxdiff = diffs.max();
        if( Props.verbose >= 2 )
        {
            if( diffs.max() > Props.tol ) 
                cout << "MaxPlus::run:  WARNING: not converged within " << Props.maxiter << " passes (" << toc() - tic << " clocks)...final maxdiff:" << diffs.max() << endl;
            else  
                cout << "MaxPlus::run:  " << "converged in " << _iterations << " passes (" << toc() - tic << " clocks)." << endl;
        }
        
        //return diffs.max();
        return bestValue;
    }


    Factor MaxPlus::beliefV( size_t i ) const {
        Prob prod( grm().var(i).states() ); 
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) 
            prod *= newMessage(i,*I);

        prod.normalize();
        return( Factor( grm().var(i), prod ) );
    }


    Factor MaxPlus::belief (const Var &n) const {
        return( beliefV( grm().findVar( n ) ) );
    }


    vector<Factor> MaxPlus::beliefs() const {
        vector<Factor> result;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            result.push_back( beliefV(i) );
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            result.push_back( beliefF(I) );
        return result;
    }


    Factor MaxPlus::belief( const VarSet &ns ) const {
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


    Factor MaxPlus::beliefF (size_t I) const {
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


    Complex MaxPlus::logZ() const {
        Complex sum = 0.0;
        for(size_t i = 0; i < grm().nrVars(); i++ )
            sum += Complex(1.0 - grm().nbV(i).size()) * beliefV(i).entropy();
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            sum -= KL_dist( beliefF(I), grm().factor(I) );
        return sum;
    }


    string MaxPlus::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }


    void MaxPlus::init( const VarSet &ns ) {
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) {
            size_t ni = grm().findVar( *n );
            for( FactorGraph::nb_cit I = grm().nbV(ni).begin(); I != grm().nbV(ni).end(); I++ )
                message(ni,*I).fill( 1.0 );
        }
    }





    void MaxPlus::SetAnyTimeResults(bool turn_on, 
            ofstream* results, ofstream* timings)
    {
        writeAnyTimeResults = turn_on;
        if(turn_on)
        {
            results_f = results;
            timings_f = timings;         
        }
        else
        {
            results_f = NULL;
            timings_f = NULL;
        }

    }
} //end namespace
