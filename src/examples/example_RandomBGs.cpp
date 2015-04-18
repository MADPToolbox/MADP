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
 *
 * For contact information please see the included AUTHORS file.
 */
#include <sstream>

#include "Globals.h"
#include "directories.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverAlternatingMaximization.h"
#include "BGIP_SolverCE.h"
#include "BGIP_SolverBranchAndBound.h"
//#include "BGIP_SolverMaxPlus.h"
#include "Timing.h"
#include "BGIP_SolverType.h"

using namespace std;
using namespace BGIP_SolverType;


/// return a BayesianGameIdenticalPayoff solver....
BayesianGameIdenticalPayoffSolver* 
NewBGIPSolver(BGIP_sharedPtr bgip, BGIP_Solver_t type)
{

    BayesianGameIdenticalPayoffSolver* p = 0;
    switch(type)
    {
    case BFS:      
        //Brute force search
        p = new BGIP_SolverBruteForceSearch<JointPolicyPureVector>(bgip, 0, 1);
        break;

    case AM:
        {
        //Alternating Maximization
        size_t nrAMrestarts = 100; 
        p = new BGIP_SolverAlternatingMaximization<JointPolicyPureVector>(bgip,
                                                                          nrAMrestarts);
        break;
        }

    case CE:
        {
        //See Frans A. Oliehoek, Julian F.P. Kooi, and Nikos Vlassis. The Cross-Entropy Method for Policy Search in Decentralized POMDPs. Informatica, 32:341–357, 2008. for explanation of parameters.
        size_t nrRestarts = 10;
        size_t nrIterations = 30;
        size_t nrSamples = 40;
        size_t nrSamplesForUpdate =15;
        bool use_hard_threshold = true; //(gamma in CE papers)
        double alpha = 0.3;
        p = new BGIP_SolverCE(bgip,
                              nrRestarts,
                              nrIterations,
                              nrSamples,
                              nrSamplesForUpdate,
                              use_hard_threshold,
                              alpha);
        break;
        }

/*  not included?       
    case MaxPlus:
        p = new BGIP_SolverMaxPlus<JointPolicyPureVector>(
            bgip,
            args.maxplus_maxiter,
            args.maxplus_updateT,
            args.maxplus_verbose,
            args.maxplus_damping);
                    break;
*/                    
    case BnB:
        {
        //The Branch and bound method from 
        //Frans A. Oliehoek, Matthijs T. J. Spaan, Jilles Dibangoye, and Christopher Amato. Heuristic Search for Identical Payoff Bayesian Games. In Proceedings of the Ninth International Conference on Autonomous Agents and Multiagent Systems, pp. 1115–1122, May 2010.
        int verbose=0;
        size_t nrDesiredSolutions=1;
        bool expandAll=false;
        BGIP_BnB::BnB_JointTypeOrdering jtOrdering=BGIP_BnB::DescendingProbability;
        bool BnB_consistentCompleteInformationHeur=true;
        p = new BGIP_SolverBranchAndBound<JointPolicyPureVector>(
            bgip, 
            verbose, 
            nrDesiredSolutions, 
            expandAll,
            jtOrdering,
            BnB_consistentCompleteInformationHeur
            );
        break;
        }

    default:        
        throw E("Incorrect BayesianGameIdenticalPayoffSolver_T type");
    }
    return(p);
}

void
SolveTheBGIP(BGIP_sharedPtr bgip)
{
    // Specify the solution method.
    // (See above and in BGIP_SolverType.h)
    BGIP_SolverType::BGIP_Solver_t method = BGIP_SolverType::AM;
    try {
        Timing timer;
        timer.Start( SoftPrint(method) );
        BayesianGameIdenticalPayoffSolver * bgip_solver = NewBGIPSolver(bgip, method);
        cout << "running " <<  SoftPrint(method) << "..."<<endl;
        cout << "...value is " << bgip_solver->Solve() << endl;
        timer.Stop(  SoftPrint(method) );
        delete bgip_solver;
    }
    catch(E& e)
    {
        e.Print();
    }
}

int main()
{
// Generate BGs of various sizes:

    vector<size_t> agents;
    agents.push_back(2);
    //agents.push_back(3);

    vector<size_t> actions;
    actions.push_back(2);
    actions.push_back(3);
    //actions.push_back(4);
    
    vector<size_t> observations;
    observations.push_back(2);
    //observations.push_back(3);
    //observations.push_back(4);
    
    for(vector<size_t>::const_iterator ai=agents.begin();
        ai!=agents.end();++ai)
        for(vector<size_t>::const_iterator nrA=actions.begin();
            nrA!=actions.end();++nrA)
            for(vector<size_t>::const_iterator nrO=observations.begin();
                nrO!=observations.end();++nrO)
                for(Index i=0;i!=10;++i)
                {
                    BayesianGameIdenticalPayoff bgip = 
                        BayesianGameIdenticalPayoff::GenerateRandomBG(
                            *ai, vector<size_t>(*ai, *nrA), vector<size_t>(*ai,*nrO)
                            );

// uncomment to write the BGs to file:                   
//                    stringstream filename;
//                    filename << "randomBG_nrAgents" << *ai
//                             << "_nrActions" << *nrA
//                             << "_nrObs" << *nrO
//                             << "_i" << i;
//                    cout << filename.str() << endl;
//                    BayesianGameIdenticalPayoff::Save(bgip,filename.str());
//
                    BGIP_sharedPtr ptr( new BayesianGameIdenticalPayoff() ); 
                    *ptr = bgip;
                    SolveTheBGIP(ptr);
                }

            
    return(0);
}
