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
 * Bas Terwijn
 *
 * For contact information please see the included AUTHORS file.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <string>

#include "Globals.h"
#include "PolicyGlobals.h"
#include "ProblemDecTiger.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverAlternatingMaximization.h"
#include "BGIP_SolverCE.h"
#include "BGIP_SolverMaxPlus.h"
#include "BGIP_SolverBranchAndBound.h"
#include "Timing.h"

#define BEEP 1

#include "argumentHandlers.h"

const char *argp_program_version = "test_BGIP_SolverCE 0.1";

// Program documentation
static char doc[] =
"This program generates a random BG and solves it with BFS, AM and CE. \
\v";

const struct argp_child childVector[] = {
//    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::CE_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

using namespace std;
using namespace BGIP_BnB;

void testBGIP_Solvers();

//the structure in which the options are put
ArgumentHandlers::Arguments args;

int main(int argc, char **argv)
{

    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    try
    {
        testBGIP_Solvers();
    }catch(E& e)
    {
        e.Print();
    }
    return(0);
}//end of main


void testBGIP_Solvers()
{
    if (!args.testMode)
        srand(time(NULL));
    else
        cout<<"running in 'testMode'"<<endl;

    /*
    size_t nrAgents = 3;
    Index acs_ar[] = {4,3,5};
    vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
    Index obs_ar[] = {3,2,4};
    vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
    */

    size_t nrAgents = 2;
    Index acs_ar[] = {2,2};
    vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
    Index obs_ar[] = {1,2};
    vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));

    BGIP_sharedPtr bgip = BGIP_sharedPtr(
        new BayesianGameIdenticalPayoff(
            BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs)));

    if(args.testMode)
    {
        bgip->SetProbability(0,0.333966);
        bgip->SetProbability(1,0.666034);

        bgip->SetUtility(0, 0, 6.80375 );
        bgip->SetUtility(0, 1, -2.11234 );
        bgip->SetUtility(0, 2, 5.66198 );
        bgip->SetUtility(0, 3,  5.9688 );
        
        bgip->SetUtility(1, 0, 8.23295 );
        bgip->SetUtility(1, 1, -6.04897 );
        bgip->SetUtility(1, 2, -3.29554 );
        bgip->SetUtility(1, 3, 5.36459 );
    }
/*
  BayesianGameIdenticalPayoff bgip(nrAgents, acs, obs);

    bgip.SetProbability(0, 0.5 );
    bgip.SetProbability(1, 0.5 );
*/
    //bgip.SetUtility(jtype, ja, real );
/*    
    bgip.SetUtility(0, 0, -1 );   // ja 0 = < 0, 0>
    bgip.SetUtility(0, 1, 5 );      // ja 1 = < 0, 1>
    bgip.SetUtility(0, 2, 15 );     // ja 2 = < 1, 0>
    bgip.SetUtility(0, 3, -1 );     // ja 3 = < 1, 1>
    
    bgip.SetUtility(1, 0, -1 );   // ja 0 = < 0, 0>
    bgip.SetUtility(1, 1, 15 );      // ja 1 = < 0, 1>
    bgip.SetUtility(1, 2, 5 );     // ja 2 = < 1, 0>
    bgip.SetUtility(1, 3, -1 );     // ja 3 = < 1, 1>
*/
/*
    bgip.SetUtility(0, 0, -4 );   // ja 0 = < 0, 0>
    bgip.SetUtility(0, 1, -2 );      // ja 1 = < 0, 1>
    bgip.SetUtility(0, 2, -8 );     // ja 2 = < 1, 0>
    bgip.SetUtility(0, 3,  2 );     // ja 3 = < 1, 1>
    
    bgip.SetUtility(1, 0, -5 );   // ja 0 = < 0, 0>
    bgip.SetUtility(1, 1, -11 );      // ja 1 = < 0, 1>
    bgip.SetUtility(1, 2, -6 );     // ja 2 = < 1, 0>
    bgip.SetUtility(1, 3,  9 );     // ja 3 = < 1, 1>
*/
//Bayesian game with 2 agents
//Number of actions < 2, 2 > (4 joint actions)
//Number of types < 2, 1 > (2 joint types)
//joint type probs: < 0.481019, 0.518981 >
//Utility function:
//-4.66505 -1.60925 -7.7225 2.12197 
//-5.10967 -8.83841 -6.37735 9.13368 
//---------------------
//BFS 5.76091
//---------------------
//  bgip.Print();

    Timing Time;    
    Time.Start("Overall");

    Time.Start("BGIP_SolverBFS");
    BGIP_SolverBruteForceSearch<JointPolicyPureVector> bfs(bgip);

    cout << "---------------------"<<endl;
    cout << "BFS " << bfs.Solve() << endl;
    cout << "---------------------"<<endl;
    const boost::shared_ptr<JointPolicy> jpolBFS=bfs.GetJointPolicy();
    jpolBFS->Print();
    Time.Stop("BGIP_SolverBFS");


#if 1
    Time.Start("BGIP_SolverAM");
    BGIP_SolverAlternatingMaximization<JointPolicyPureVector> am(bgip);
    cout << "---------------------"<<endl;
    cout << "AM " << am.Solve() << endl;
    cout << "---------------------"<<endl;
    const boost::shared_ptr<JointPolicy> jpolAM=am.GetJointPolicy();
    jpolAM->Print();    
    Time.Stop("BGIP_SolverAM");

    Time.Start("BGIP_SolverCE");
    BGIP_SolverCE ce(bgip,
        args.nrCERestarts,
        args.nrCEIterations,
        args.nrCESamples,
        args.nrCESamplesForUpdate,
        args.CE_use_hard_threshold,
        args.CE_alpha
            );
    cout << "---------------------"<<endl;
    cout << "CE " << ce.Solve() << endl;
    cout << "---------------------"<<endl;
    const boost::shared_ptr<JointPolicy> jpolCE=ce.GetJointPolicy();
    jpolCE->Print();
    cout << endl;
    Time.Stop("BGIP_SolverCE");
#endif                 

#if 0
    Time.Start("BGIP_SolverMaxPlus");
    BGIP_SolverMaxPlus<JointPolicyPureVector> mp(bgip,
            1000,
            string("PARALL"),
            9
            );
    cout << "---------------------"<<endl;
    cout << "MaxPlus " << mp.Solve() << endl;
    cout << "---------------------"<<endl;
    const boost::shared_ptr<JointPolicy> jpolMaxPlus=mp.GetJointPolicy();
    jpolMaxPlus->Print();
    cout << endl;
    Time.Stop("BGIP_SolverMaxPlus");    
    Time.Stop("Overall");
    if(args.verbose >= 0 && !args.testMode)
        Time.PrintSummary();
#endif


    for(Index i=0;i!=NUMBER_OF_BNB_JOINT_TYPE_ORDERINGS;++i)
    {
        stringstream ss;
        BnB_JointTypeOrdering jto=static_cast<BnB_JointTypeOrdering>(i);
        ss << "BGIP_SolverBranchAndBound_" << SoftPrint(jto);
        Time.Start(ss.str());
        BGIP_SolverBranchAndBound<JointPolicyPureVector> bnb(bgip,args.verbose,INT_MAX,false,jto);
        cout << "---------------------"<<endl;
        cout << ss.str() << " " << bnb.Solve() << endl;
        cout << "---------------------"<<endl;
        bnb.GetJointPolicyPureVector().Print();
        cout << endl;
        Time.Stop(ss.str());    
    }

    Time.Stop("Overall");
    if(args.verbose >= 0 && !args.testMode)
        Time.PrintSummary();
}
