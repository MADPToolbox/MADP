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

#include "ndp.h"
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

    const char *NDP::Name = "NDP";

    bool NDP::initProps() {
        //if( !HasProperty("updates") )
            //return false;
        //if( !HasProperty("tol") )
            //return false;
        //if( !HasProperty("maxiter") )
            //return false;
        if( !HasProperty("verbose") )
            return false;
        
        //Props.updates = FromStringTo<UpdateType>("updates");
        //Props.tol     = FromStringTo<double>("tol");
        //Props.maxiter = FromStringTo<size_t>("maxiter");
        Props.verbose = FromStringTo<size_t>("verbose");
        //if( HasProperty("damping") )
            //Props.damping = FromStringTo<double>("damping");
        //else
            //Props.damping = 0.0;

        return true;
    }


    NDP::NDP(const FactorGraph & fg, const Properties &opts, size_t k ) 
        : 
            DAIAlgFG(fg, opts),
            Props(),
            //_maxdiff(0.0),
            //_iterations(0UL),
            //_indices(),
            //_messages(),
            //_newmessages(),
            _m_induced_width(0),
            _m_varOrder( fg.nrVars() ),
            _m_deadlineInSeconds(0),
            _m_newFacs(  fg.nrVars() ),
            _m_bestResps(fg.nrVars() ),
            _m_neighV(   fg.nrVars() ),
            //original factors + the ones that will be introduced.
            _m_neighF(   fg.nrFactors() + fg.nrVars() ),
            _g(&fg) ,
            _m_bestConfiguration( fg.nrVars() ),
            _k(k),
            _k_th_Val(-DBL_MAX)//,
            //writeAnyTimeResults(false),
            //results_f(NULL),
            //timings_f(NULL)
    {
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);

        for(size_t i=0; i < fg.nrVars() ; i++)
            //initialize neighborhoods from original model
            _m_neighV.at(i) = fg.nbV(i);
        
        for(size_t i=0; i < fg.nrFactors() ; i++)
            //initialize neighborhoods from original model
            _m_neighF.at(i) = fg.nbF(i);
            //the neighborhoods for the 'new' factors remain uninitialized.

/*        
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
*/        
    }


    void NDP::init() { //messages (one for each edge?) are initialized with uniform probabilities
        if( !initProps() )
            DAI_THROW(NOT_ALL_PROPERTIES_SPECIFIED);
        /*
        for( vector<Prob>::iterator mij = _messages.begin(); mij != _messages.end(); mij++ )
            //mij->fill(1.0 / mij->size());
            //mij->fill(0.0 ); //FRANS Max-plus initialized on 0.0
            mij->fill( (double) rand() * 50 / RAND_MAX ); //FRANS Max-plus initialized on 0.0
        _newmessages = _messages;
        */
        if( Props.verbose >= 1)
            cout << "NDP initialized at verbose="<<Props.verbose<<endl;
    }



    // NDP::run does not check for NANs for performance reasons
    // Somehow NaNs do not often occur in NDP...
    double NDP::run() {
        const FactorGraph& g = grm();
        bestValue = -INFINITY;
        if( Props.verbose >= 2 )
            cout << "Starting " << identify() << "..."<<endl;
        if( Props.verbose >= 3)
           cout << endl; 

        tms ts_before, ts_after;
        times(&ts_before);

        //struct timeval start_time, cur_time;
        //if(gettimeofday(&start_time, NULL) != 0)
            //throw "Error with gettimeofday";
        //clock_t tic = toc();

        // 0) some initialization

        // 1) create a heuristic ordering of the variables to eliminate
        ComputeHeuristicVariableOrdering();
        
        // 2) eliminate the variables one by one
        for(size_t i=0; i < g.nrVars(); i++)
        {
            // make sure we don't use too much time
            if(_m_deadlineInSeconds>0)
            {
                times(&ts_after);
                clock_t utime = ts_after.tms_utime -
                    ts_before.tms_utime;
                double timeSpentInS=static_cast<double>(utime) / sysconf(_SC_CLK_TCK);
                if(timeSpentInS>_m_deadlineInSeconds)
                    DAI_THROW(INTERNAL_ERROR);
            }
            EliminateVariable(i);
        }
        // 3) construct the k optimal assignments
        double val = ConstructOptimalAssignments();
        if(Props.verbose >= 1)
            cout << "NDP done. Induced width was "<<_m_induced_width<<endl;
        
        return(val);
    }
    //typedef  int (Fred::*FredMemFn)(char x, float y); 
    //typedef bool (NDP::*NDPMemFn)(Var i, Var j); 
    void NDP::ComputeHeuristicVariableOrdering()
    {
        //NDPMemFn vhld = &NDP::VariableHasLowerDegree;
        std::vector< Var > v = grm().vars();
        //variable with lower degrees first:
        VariableHasLowerDegree vhld(grm());
        std::sort(v.begin(), v.end(), vhld);

        _m_varOrder.resize( v.size() );
        for(size_t i=0; i<v.size(); i++)
        {
            size_t varI = grm().findVar( v.at(i) );
            _m_varOrder.at(i) = varI;
        }

        if( Props.verbose > 3)
        {
            cout << "variable ordering computed:\n[";
            for(size_t i=0; i<v.size(); i++)
                cout <<  _m_varOrder.at(i) << ", ";
            cout << "]"<<endl;
        }

    }

    void NDP::EliminateVariable(size_t i_th_var)
    {
        if(Props.verbose >= 1)
            cout << "Starting elimination of "<<i_th_var<<"-th var..."<<endl;

        size_t varIndex = _m_varOrder.at(i_th_var);
        const FactorGraph& g = grm();

        //this has been replaced by the GetFactor function that 
        //generalizes to refer to *all* (orig+ new) factors
        //const std::vector < Factor > & 	facs = g.factors();

        //get ref the the variable 
        const Var & v = g.var(varIndex);

        //get all the factors of the var - use the local 'GetNeighsV' function!
        //(since that include newly constructed factors, and only factors
        //that are still in consideration)
        const FactorGraph::nb_type &  neighFIs = GetNeighsV(varIndex);
        size_t num_Fs = neighFIs.size();
            if(Props.verbose >= 1)

        vector<size_t> neighVIs2;

        //get all the (indices of) neighbors of those, the 'neighVIs2' 
        //!??!? I used 'neighVIs' but that gave compiler errors?!?
        nb_type neighVIs2;
        //cout << printNB(neighVIs2);
        neighVIs2 = GetVNeighsV(varIndex);
        //get all the neighbors of those, the 'neighVs' 
        VarSet neighVs = VarSetFromNB(neighVIs2);
        size_t num_Vs = neighVs.size();
        if(Props.verbose >= 2)
        {
            cout << "it praticipates in "<<num_Fs << " factors";
            if(Props.verbose >= 3)
                cout << ": "<< printNB(neighFIs);
            cout <<endl;
            cout << "it has "<<num_Vs << " variables neighbors";
            if(Props.verbose >= 3)
                cout << ": "<< printNB(neighVIs2);
            cout <<endl;
        }
        if(num_Vs > _m_induced_width)
        {
            _m_induced_width = num_Vs;
            if(Props.verbose >= 1)
                cout << "Induced width increased to "<<_m_induced_width<<endl;
        }

        //the new factor will include all these neighVs:
        _m_newFacs.at(i_th_var) = Factor(neighVs);
        Factor& newFactor = _m_newFacs.at(i_th_var);
        SetNeighF(i_th_var, neighVs);

        //and we will need to keep track of the best-responses
        //of this variable v
        _m_bestResps.at(i_th_var).resize( newFactor.stateSpace() );

        //loop over all states of neighVs, compute v's best response
        //and store as a new factor.
        
        //make an index(vs_F, neighVs) for each factor F in neighFs
        vector< Index > index_vec(num_Fs);
        vector< VarSet > vs_vec(num_Fs);
        for(size_t i=0; i < num_Fs; i++)
        {
            size_t fI = neighFIs.at(i);
            const Factor& F = GetFactor( fI );
            VarSet vs_F = F.vars();
            //remove variable varIndex:
            vs_F /= v;
            vs_vec.at(i) = vs_F;
            index_vec.at(i) = Index(vs_F, neighVs);
        }
        for( size_t jI=0; jI < neighVs.stateSpace(); jI++)
        {//compute best response against joint state jI.
            //here we will store the amount that v can achieve by
            //selecting each of its states when neighVs have state jI:
            Factor contribution_vec_v(v, 0.0);

            //loop over all neighFs
            //-get the slice that corresponds to the assignment of 'states'
            // according to jI (this should be a vector of v's states)
            //-add to contribution_vec_v 
            for(size_t i=0; i < num_Fs; i++)
            {
                //process the i-th neighboring factor, corresponding to 
                size_t fI = neighFIs.at(i);
                const Factor& f = GetFactor( fI );
                VarSet & vs_i = vs_vec.at(i);
                Index & index_i = index_vec.at(i);
                Factor sliced_Fi = f.slice( vs_i, index_i);
                //sliced_Fi should be a factor depending only on v now...
                //and thus we add it to the contribution_vec_v
                contribution_vec_v += sliced_Fi;
                
                //update its index:
                ++index_i;
            }

            //sanity check:
            if( contribution_vec_v.stateSpace() != v.states() )
            {
                stringstream ss;
                ss << "contribution_vec_v.stateSpace() = "<<
                   contribution_vec_v.stateSpace() <<
                   " != v.states() = "<< 
                   v.states() << endl;

                throw ( ss.str().c_str() );
            }

            //now the state of v that has the highest value in 
            //contribution_vec_v is the best response:
            double val = -DBL_MAX;
            size_t brI = 0;
            for(size_t i=0; i < v.states() ; i++)
                if( contribution_vec_v[i] > val)
                {
                    val = contribution_vec_v[i];
                    brI = i;
                }
            
            newFactor[jI] = val;
            _m_bestResps.at(i_th_var).at(jI) = brI;

            if(Props.verbose >= 4)
                cout << "best-response for "<<jI<<"-th 'local state' is "<<
                    _m_bestResps.at(i_th_var).at(jI) << " (value = "<<
                    newFactor[jI] <<")"<<endl;

        }
        if(Props.verbose >= 7)
            cout << "New factor (and best-responses) computed, updating neighbors..."<<
               endl; 

        //update: _m_neighV
        //-> the factors with indices 'neighFIs' have been removed
        //-> these factors have been replaced by 
        //      'newFactor = _m_newFacs.at(i_th_var)'
        //   whose global index is given by 
        //      gI = ToGlobalFactorIndex(i_th_var)
        //
        //-> the variables in Varset  'neighVs'  are affected by that
        //      - remove old factor indices
        //      - add gI
        
        for(size_t i=0; i < neighVIs2.size(); i++)
        {
            size_t vI = neighVIs2.at(i);
            nb_type & nb_vI = _m_neighV.at(vI);
            
            //add new factor to nb_vI
            size_t gI = ToGlobalFactorIndex(i_th_var);
            nb_vI.push_back(gI);

            //remove indices from neighFIs
            for(size_t j=0; j<neighFIs.size(); j++)
            {
                size_t fI = neighFIs.at(j);
                //check if fI in nb_vI
                //     std::find(vector.begin(), vector.end(), item)!=vector.end()
                nb_type::iterator pos = std::find(
                        nb_vI.begin(),
                        nb_vI.end(),
                        fI);
                //i guess it will do not harm if pos==end() ?
                //nb_vI.erase(pos);
                //well the segfault indicates otherwise:
                if(pos != nb_vI.end())
                    nb_vI.erase(pos);
            }
        }

        //update _m_neighF
        //-> remove this agent from any neighborhood
        //-> but is not necessary... all its factors have been removed!

        if(Props.verbose >= 2)
            cout << i_th_var<<"-th variable (varIndex="<<varIndex<<
                ") has been eliminated"<<endl;

    }

    double NDP::ConstructOptimalAssignments()
    {

        size_t lastVarI =  _g->nrVars()-1;

        //the optimal value is directly given by the factor introduced for
        //the very last variable! - which has 1 local state (the empty one)
        double value = _m_newFacs.at(lastVarI)[0];
        
        if(Props.verbose >= 1)
            cout << "NDP::ConstructOptimalAssignments() - value="<<value<<endl;

        //traverse the variables in reverse order:
        for(size_t i=0; i <= lastVarI; i++)
        {
            //reverse order index:
            size_t rI = lastVarI - i;
            SelectStateForVar(rI);
        }

        valConf solution = make_pair(value, _m_bestConfiguration );
        _valConfs.push_front(solution);

        return(value);
    }
    void NDP::SelectStateForVar(size_t varOrderI)
    {
        //get the factor that was introduced when eliminating varOrderI
        Factor& fI = _m_newFacs.at(varOrderI);
        
        //get the variables involved
        const VarSet & vs = fI.vars();
        nb_type vsI = NBFromVarSet(vs);
        
        //see what states they have selected
        std::vector<size_t> indivIndices( vsI.size() );
        for(size_t i=0; i < vsI.size(); i++)
        {
            size_t nvI = vsI.at(i);
            indivIndices.at(i) = _m_bestConfiguration.at(nvI);
        }

        //compute the joint state
        multind mi(vs);
        size_t jI = mi.li( indivIndices );
        //size_t jI2 = Factor::IndividualToJointFactorIndex(vs, indivIndices);
        //if (jI != jI2)
            //cout << "(jI != jI2): " << jI <<"!="<< jI2 << endl;

        //look up varOrderI's best response
        size_t br = _m_bestResps.at(varOrderI).at(jI);

        //register varOrderI's choice
        size_t varI = _m_varOrder.at(varOrderI);
        _m_bestConfiguration.at(varI) = br;
    }


/*

    Factor NDP::beliefV( size_t i ) const {
        Prob prod( grm().var(i).states() ); 
        for( FactorGraph::nb_cit I = grm().nbV(i).begin(); I != grm().nbV(i).end(); I++ ) 
            prod *= newMessage(i,*I);

        prod.normalize();
        return( Factor( grm().var(i), prod ) );
    }
*/

    Factor NDP::belief (const Var &n) const {
        throw("makes no sense?");
        Factor f; 
        return(f);
        //return( beliefV( grm().findVar( n ) ) );        
    }


    vector<Factor> NDP::beliefs() const {
        throw("makes no sense?");
        vector<Factor> f; 
        return(f);
/*        
        vector<Factor> result;
        for( size_t i = 0; i < grm().nrVars(); i++ )
            result.push_back( beliefV(i) );
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            result.push_back( beliefF(I) );
        return result;
*/
    }

    Factor NDP::belief( const VarSet &ns ) const {
        throw("makes no sense?");
        Factor f; 
        return(f);
/*        
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
*/        
    }

/*
    Factor NDP::beliefF (size_t I) const {
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
#if 0 
        UNOPTIMIZED VERSION
     
        Factor prod( grm().factor(I) );
        for( FactorGraph::nb_cit i = grm().nbF(I).begin(); i != grm().nbF(I).end(); i++ ) {
            for( FactorGraph::nb_cit J = grm().nbV(*i).begin(); J != grm().nbV(*i).end(); J++ )
                if( *J != I )
                    prod *= Factor( grm().var(*i), newMessage(*i,*J)) );
        }
        return prod.normalized();
#endif        
    }
*/    

    Complex NDP::logZ() const {
        throw("NDP::logZ makes no sense?");
        Complex sum = 0.0;
/*        
        for(size_t i = 0; i < grm().nrVars(); i++ )
            sum += Complex(1.0 - grm().nbV(i).size()) * beliefV(i).entropy();
        for( size_t I = 0; I < grm().nrFactors(); I++ )
            sum -= KL_dist( beliefF(I), grm().factor(I) );
*/            
        return sum;
    }

    string NDP::identify() const { 
        stringstream result (stringstream::out);
        result << Name << GetProperties();
        return result.str();
    }

    void NDP::init( const VarSet &ns ) {
/*        
        for( VarSet::const_iterator n = ns.begin(); n != ns.end(); n++ ) {
            size_t ni = grm().findVar( *n );
            for( FactorGraph::nb_cit I = grm().nbV(ni).begin(); I != grm().nbV(ni).end(); I++ )
                message(ni,*I).fill( 1.0 );
        }
*/        
    }



/*
    void NDP::SetAnyTimeResults(bool turn_on, 
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
*/

} //end namespace
