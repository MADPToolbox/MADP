/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "MaxPlusSolverForBGs.h"

#include <float.h>
#include <iostream>
#include <fstream>
#include "var.h"
#include "maxplus.h"
#include "BGIP_SolverMaxPlus.h"
#include "BayesianGameIdenticalPayoffInterface.h"

using namespace std;

//Default constructor
MaxPlusSolverForBGs::MaxPlusSolverForBGs(
        size_t maxiter,
        string updateType,
        int verbosity,
        double damping,
        size_t nrSolutions,
        size_t nrRestarts)
    :
    MaxPlusSolver(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts)
{
}

/*
//Copy constructor.    
MaxPlusSolverForBGs::MaxPlusSolverForBGs(const MaxPlusSolverForBGs& o) 
{
}
//Destructor
MaxPlusSolverForBGs::~MaxPlusSolverForBGs()
{
}
//Copy assignment operator
MaxPlusSolverForBGs& MaxPlusSolverForBGs::operator= (const MaxPlusSolverForBGs& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/

void MaxPlusSolverForBGs::Construct_AgentTypePair_Variables(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bgip, 
        vector< vector<Index> >& var_indices, vector< libDAI::Var >& vars, 
        int verbosity
        )    
{    
    var_indices.resize( bgip->GetNrAgents() );
    size_t nrATs=0;
    vector<size_t> nrTypes(bgip->GetNrAgents() );
    for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
    {
        nrTypes[agI] =  bgip->GetNrTypes(agI);
        if(verbosity >= 2)
            cout << "Agent "<<agI<<" has " << nrTypes[agI] << " types."<<endl;
        nrATs += nrTypes[agI];
        var_indices[agI].reserve( bgip->GetNrTypes(agI) );
    }
    if(verbosity >= 1)
        cout << "FactorGraph has "<<nrATs<<" variables (agent-types)"<<endl;
   
    //reserve memory 
    for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
        var_indices[agI].reserve( nrTypes[agI] );
    vars.reserve(nrATs);

    Index atI = 0;
    for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
        for(Index tI = 0; tI < bgip->GetNrTypes(agI); tI++)
        {
            //Note variables for higher agents have higher indices!
            libDAI::Var v(atI, bgip->GetNrActions(agI));
            var_indices[agI].push_back(atI);
            vars.push_back(v);
            atI++;
        }
/*    //not needed:
    Index nrAgentTypes = atI;
    if(debug)
    {
        cout << "Created "<<nrAgentTypes<<" variables for the factor graph" <<
            ", var_indices[agI][tI]="<< SoftPrintVector(var_indices) << endl;
    }
*/    
}

void MaxPlusSolverForBGs::Construct_JointType_Factors(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bgip,
        const vector< vector<Index> >& var_indices,
        const vector< libDAI::Var >& vars,
        vector<libDAI::Factor>& facs,
        int verbosity
        )
{
    const size_t nrJA =  bgip->GetNrJointActions();
    size_t nrFacs =  bgip->GetNrJointTypes() ;
    facs.reserve(nrFacs);

    if(verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors"<<endl;
    //factor values are copied by value (?) so this is not needed:
    //vector<double **> factor_vals( bgip->GetNrJointTypes() );
    for(Index jtI=0; jtI < bgip->GetNrJointTypes(); jtI++)
    {
        vector<Index> indTypes = bgip->JointToIndividualTypeIndices(jtI);
        if(verbosity > 4)
            cout << "Adding factor for joint type jtI="<<jtI<<" ( indtypes="<<
                SoftPrintVector(indTypes) << " )"<<endl;

        //compute the agent-type (i.e., factor) indices atI, get the
        //relevant variables and put them in a VarSet
        libDAI::VarSet vs;
        for(Index agI=0; agI < bgip->GetNrAgents(); agI++)
        {

            Index atI = var_indices[agI][indTypes[agI]];
            vs = vs | vars[atI];
            if(verbosity > 5)
                cout << "agent "<<agI<<" contributes through variable atI="<<atI<<endl;

        }

        libDAI::Real factor_vals[nrJA];
        for(Index jaI=0; jaI < nrJA; jaI++)
        {
            vector<Index> indAcs = bgip->JointToIndividualActionIndices(jaI);

            //libdai conversion below needs size_t arguments:
            vector<size_t> indAcsST(indAcs.size());
            for(Index i=0; i<indAcs.size(); i++)  
                indAcsST[i] = (size_t) indAcs[i];  

            if(verbosity > 6)
                cout << "Computing joint factor index (JFI) for vs="<<vs<<", (action)indices="<<
                    SoftPrintVector(indAcs) <<endl;

            Index jointFactorIndex = libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
            if(verbosity > 6)
                cout << "JFI="<<jointFactorIndex<<endl;
            /* this doesn't work:
            // in our implementation (actions of) higher numbered agents are 
            // the least significant, but in libDAI higher numbered variables
            // (corresponding to higher numbered agents, see above) are the most
            // significant. Therefore this is done in reverse:
            Index rev_index = nrJA - jaI - 1;
            */
            double perturbation=(1e-6*rand())/RAND_MAX;
            factor_vals[jointFactorIndex] =
                (bgip->GetUtility(jtI, jaI) + perturbation ) *
                bgip->GetProbability(jtI);
        }

        libDAI::Factor f(vs, factor_vals);
        if(verbosity > 4)
        {
            cout << "Added factor f:"<< f << endl;
        }
        facs.push_back(f);
    }
}
