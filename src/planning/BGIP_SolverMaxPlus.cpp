/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include <float.h>
#include <iostream>
#include <fstream>
#include "var.h"
#include "maxplus.h"
//#include "alldai.h"
#include "BGIP_SolverMaxPlus.h"

using namespace std;

#define DEBUG_BGIP_SOLVER_BFS 0
//#define DEBUG_EXHBR 0
//#define DEBUG_EXJESP 0

#define DEBUG_BGIP_SOLVER_BFS_PRINTOUTPROGRESS 0

#if 0
//Default constructor
BGIP_SolverMaxPlus::BGIP_SolverMaxPlus(
        const BayesianGameIdenticalPayoffInterface& bg,
        size_t maxiter,
        string updateType,
        size_t verbosity,
        double damping,
        size_t nrSolutions,
        size_t nrRestarts

        ) :
    BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>(bg),
    MaxPlusSolverForBGs(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts)
    //_m_maxiter(maxiter),
    //_m_updateType(updateType),
    //_m_verbosity(verbosity),
    //_m_damping(damping),
    //_m_nrSolutions(nrSolutions),
    //_m_nrRestarts(nrRestarts)
{}


double BGIP_SolverMaxPlus::Solve()
{ 
    const BayesianGameIdenticalPayoffInterface * bgip = GetReferred();

    //an assignment of variables (v3, v8, v14) corresponds to an assignment
    //of joint actions (a1, a2, a3) for a particular joint type.
    //I.e., each (the action taken for eah) (agent,type)-pair is a variable 
    //of the factor graph.
    //let's create these variables
    
    // var_indices[agI][tI] stores the (agent,type)-pair index atI
    vector< vector<Index> > var_indices;    
    // vars stores the variables. vars[atI]
    vector< libDAI::Var > vars;
    bool debug = (_m_verbosity > 7);
    Construct_AgentTypePair_Variables(bgip, var_indices, vars, debug);

    //each joint type corresponds to a factor... 
    //let's create them
    vector<libDAI::Factor> facs;
    Construct_JointType_Factors(bgip, var_indices, vars, facs, debug);

    //and finally you construct the FactorGraph from that:
    libDAI::FactorGraph fg(facs);

    //size_t  maxiter = 1000;
    //size_t  verb = 2;
    libDAI::Properties props;
    props.Set("maxiter",_m_maxiter);
    props.Set("verbose",_m_verbosity);
    props.Set("updates",_m_updateType);
    props.Set("damping",_m_damping);
    double  tol = 1e-4;
    props.Set("tol",tol);
    libDAI::MaxPlus mp (fg, props);
    mp.init();
    if(_m_writeAnyTimeResults)
        mp.SetAnyTimeResults(true, _m_results_f, _m_timings_f);
    double value = mp.run();

    //Create the BG policy as computed by MaxPlus...
    
    const std::vector<size_t> & config = mp.GetBestConfiguration();
    JointPolicyPureVector jpolBG( _m_solution.GetJointPolicyPureVector() );
    for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
        for(Index tI = 0; tI < bgip->GetNrTypes(agI); tI++)
        {
            Index varIndex = var_indices[agI][tI];
            Index bestAction = config[varIndex];
            jpolBG.SetAction(agI, tI, bestAction);
        }
    _m_solution.SetPolicy(jpolBG);

    //store the solution
    _m_solution.AddSolution( *(new JointPolicyPureVector(jpolBG)),
                             value);

    return(value);



}

#endif
/*
double BGIP_SolverMaxPlus::OLDSolve()
{ 
    libDAI::Var a;
    BayesianGameIdenticalPayoffInterface * bgip = GetReferred();

    //an assignment of variables (v3, v8, v14) corresponds to an assignment
    //of joint actions (a1, a2, a3) for a particular joint type.

    //each (agent,type)-pair is a variable of the factor graph
    //let's create them
    //
    // var_indices[agI][tI] stores the (agent,type)-pair index atI
    vector< vector<Index> > var_indices( bgip->GetNrAgents() );
    // vars stores the variables. vars[atI]
    vector< libDAI::Var > vars;
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
    //not needed:
    Index nrAgentTypes = atI;
    if(_m_verbosity > 7)
    {
        cout << "Created "<<nrAgentTypes<<" variables for the factor graph" <<
            ", var_indices[agI][tI]="<< SoftPrintVector(var_indices) << endl;
    }


    //each joint type corresponds to a factor... 
    //let's create them
    vector<libDAI::Factor> facs;
    const size_t nrJA =  bgip->GetNrJointActions();
    //factor values are copied by value (?) so this is not needed:
    //vector<double **> factor_vals( bgip->GetNrJointTypes() );
    for(Index jtI=0; jtI < bgip->GetNrJointTypes(); jtI++)
    {
        vector<Index> indTypes = bgip->JointToIndividualTypeIndices(jtI);
        if(_m_verbosity > 7)
            cout << "Adding factor for joint type jtI="<<jtI<<" ( indtypes="<<
                SoftPrintVector(indTypes) << " )"<<endl;

        //compute the agent-type (i.e., factor) indices atI, get the
        //relevant variables and put them in a VarSet
        libDAI::VarSet vs;
        for(Index agI=0; agI < bgip->GetNrAgents(); agI++)
        {

            atI = var_indices[agI][indTypes[agI]];
            vs = vs | vars[atI];
            if(_m_verbosity > 7)
                cout << "agent "<<agI<<" contributes through variable atI="<<atI<<endl;

        }

        libDAI::Real factor_vals[nrJA];
        for(Index jaI=0; jaI < nrJA; jaI++)
        {
            vector<Index> indAcs = bgip->JointToIndividualActionIndices(jaI);
            vector<size_t> indAcsST(indAcs.size());
            for(Index i=0; i<indAcs.size(); i++)  indAcsST[i] = (size_t) indAcs[i];  
            if(_m_verbosity > 7)
                cout << "Computing joint factor index (JFI) for vs="<<vs<<", (action)indices="<<
                    SoftPrintVector(indAcs) <<endl;
            Index jointFactorIndex = libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
            if(_m_verbosity > 7)
                cout << "JFI="<<jointFactorIndex<<endl;
            // this doesn't work:
            // in our implementation (actions of) higher numbered agents are 
            // the least significant, but in libDAI higher numbered variables
            // (corresponding to higher numbered agents, see above) are the most
            // significant. Therefore this is done in reverse:
            // Index rev_index = nrJA - jaI - 1;
            //
            factor_vals[jointFactorIndex] = bgip->GetUtility(jtI, jaI) *
                bgip->GetProbability(jtI);
        }

        libDAI::Factor f(vs, factor_vals);
        if(_m_verbosity > 7)
        {
            cout << "Added factor f:"<< f << endl;
        }
        facs.push_back(f);
    }

    //and finally you construct the FactorGraph from that:
    libDAI::FactorGraph fg(facs);

    //size_t  maxiter = 1000;


    //size_t  verb = 2;
    libDAI::Properties props;
    props.Set("maxiter",_m_maxiter);
    props.Set("verbose",_m_verbosity);
    //props.Set("updates",string("SEQFIX"));
    //props.Set("updates",string("PARALL"));
    props.Set("updates",_m_updateType);
    props.Set("damping",_m_damping);
    double  tol = 1e-4;
    props.Set("tol",tol);
    libDAI::MaxPlus mp (fg, props);
    mp.init();
    if(_m_writeAnyTimeResults)
        mp.SetAnyTimeResults(true, _m_results_f, _m_timings_f);
    double value = mp.run();

    //Create the BG policy as computed by MaxPlus...
    //
    //get the configuration
    const std::vector<size_t> & config = mp.GetBestConfiguration();
    //create joint policy
    JointPolicyPureVector jpolBG( _m_solution.GetJointPolicyPureVector() );
    for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
        for(Index tI = 0; tI < bgip->GetNrTypes(agI); tI++)
        {
            Index varIndex = var_indices[agI][tI];
            Index bestAction = config[varIndex];
            jpolBG.SetAction(agI, tI, bestAction);
        }
            
    _m_solution.SetPolicy(jpolBG);

    //store the solution
    JPPVValPair * temp =  new JPPVValPair( 
        new JointPolicyPureVector(jpolBG), // make a copy! (jpolBG is local)
        value );

    _m_solution.AddSolution( temp );

    return(value);



}

*/
