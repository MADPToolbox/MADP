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
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverAlternatingMaximization.h"
#include "BGIP_SolverCE.h"
#include "BGIP_SolverMaxPlus.h"
#include "BGCG_SolverMaxPlus.h"
#include "BGIP_SolverBranchAndBound.h"
#include "Timing.h"

#include "Problem_CGBG_FF.h"

#define BEEP 1

#include "argumentHandlers.h"

const char *argp_program_version = "test_... 0.1";

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
//#include "boost/math/constants/constants.hpp"
//#include "boost/math/special_functions/erf.hpp"

using namespace std;
using namespace BGIP_BnB;

//using namespace boost::math;

void testBGIP_Solvers();

//the structure in whiuch the options are put
ArgumentHandlers::Arguments args;

void test_CGBG_FF()
{
    //size_t nr_types[] = {3,4,5,6,7,8,9,10,15,20,25,30,40,};
    //size_t nr_types[] = {50};
    //vector<size_t> nrTypesPerAgent(nr_types, nr_types + sizeof(nr_types)/sizeof(size_t));
    //for(Index i = 0; i < nrTypesPerAgent.size(); i++)
        //test_WithNTypes(nrTypesPerAgent.at(i));
        //
        //
    Timing Time;    
    
    /* - problem case: number of joint types overflow: */
    size_t nrHouses = 144;
    size_t nrFLs = 3;
    size_t nrAgents = 40;
    size_t nrActionsPerAgent = 3;       //i.e., z
    size_t nrObservedHousesPerAgent = 2;//i.e., y
    size_t k =2;                        //the maximum number of agents in a house's payoff function
    /* */
    /* - problem case: fixed *
    size_t nrHouses = 5;
    size_t nrFLs = 2;
    size_t nrAgents = 5;
    size_t nrActionsPerAgent = 2;       //i.e., z
    size_t nrObservedHousesPerAgent = 2;//i.e., y
    size_t k =2;                        //the maximum number of agents in a house's payoff function
    / * * /
    size_t nrHouses = 3;
    size_t nrFLs = 2;
    size_t nrAgents = 2;
    size_t nrActionsPerAgent = 1;       //i.e., z
    size_t nrObservedHousesPerAgent = 1;//i.e., y
    size_t k =2;                        //the maximum number of agents in a house's payoff function
    cout << "Initializing the CGBG generalized FF problem..."<<endl;
    / * */
    srand(0);
    BGCG_sharedPtr bg = BGCG_sharedPtr(
        new Problem_CGBG_FF(
                nrHouses,
                nrFLs,
                nrAgents,
                nrActionsPerAgent,  
                nrObservedHousesPerAgent,   
                k
            )
        );
    cout <<"Problem created:"<<endl;
    cout << bg->SoftPrint() << endl;
    bg->SanityCheckBGCG();

    cout << "\n\n------------------------------------------------------"<<endl;
    cout << "--STARTING SOLVERS------------------------------------"<<endl;
    cout << "------------------------------------------------------\n\n"<<endl;

    //BGCG Max Plus
    Time.Start("BGCG_MaxPlus");
    size_t maxiter = 1000;
    std::string updateType=std::string("PARALL");
    size_t verbosity = 2;
    double damping = 0.0;
    size_t nrSolutions = 1;
    size_t nrRestarts = 1;
    BGCG_SolverMaxPlus* bgcg_maxplus = 
        new BGCG_SolverMaxPlus(
            bg, maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts
        );
    cout << "BGCG_MaxPlus " << bgcg_maxplus->Solve() << endl;
    cout << "---------------------" << endl;
    boost::shared_ptr<JointPolicy> jpolBGCG_MaxPlus=bgcg_maxplus->GetJointPolicy();
    jpolBGCG_MaxPlus->Print();    
    Time.Start("BGCG_MaxPlus");


    Time.Start("BGIP_MaxPlus");
    //size_t maxiter = 1000;
    //std::string updateType=std::string("PARALL");
    //size_t verbosity = 2;
    //double damping = 0.0;
    //size_t nrSolutions = 1;
    //size_t nrRestarts = 1;
    BGIP_SolverMaxPlus<JointPolicyPureVector>* bgip_maxplus = 
        new BGIP_SolverMaxPlus<JointPolicyPureVector>(
            bg, maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts
        );
    cout << "BGIP_MaxPlus " << bgip_maxplus->Solve() << endl;
    cout << "---------------------" << endl;
    boost::shared_ptr<JointPolicy> jpolBGIP_MaxPlus=bgip_maxplus->GetJointPolicy();
    jpolBGIP_MaxPlus->Print();    
    Time.Start("BGIP_MaxPlus");
/*
    // BAGABAB
    Time.Start("BGIP_SolverBaGaBaB");
    int verbose=0;
    size_t nrSolutions=1;
    bool keepAll=false;
    //BGIP_BnB::BnB_JointTypeOrdering ordering=BGIP_BnB::IdentityMapping;
    BGIP_BnB::BnB_JointTypeOrdering ordering=BGIP_BnB::DescendingProbability;
    bool reComputeHeur=false;
    //operators:
    BGIP_SolverBranchAndBound<JointPolicyPureVector>* bagabab =
        new BGIP_SolverBranchAndBound<JointPolicyPureVector>(
            bg,
            verbose,
            nrSolutions,
            keepAll,
            ordering,
            reComputeHeur
            );
    double deadlineInSeconds=5;
    //bagabab->SetDeadline(deadlineInSeconds);

    cout << "BAGABAB " << bagabab->Solve() << endl;
    cout << "---------------------"<<endl;
    boost::shared_ptr<JointPolicy> jpolBAGABAB=bagabab->GetJointPolicy();
    jpolBAGABAB->Print();    
    
    Time.Stop("BGIP_SolverBaGaBaB");
*/

    Time.Start("BGIP_SolverAM");
    BGIP_SolverAlternatingMaximization<JointPolicyPureVector> am(bg, 5);
    for (Index r=0; r < 1; r++)
        cout << "AM " << am.Solve() << endl;
    cout << "---------------------"<<endl;
    boost::shared_ptr<JointPolicy> jpolAM=am.GetJointPolicy();
    jpolAM->Print();    
    Time.Stop("BGIP_SolverAM");
    
}

int main(int argc, char **argv)
{



    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    try
    {
        srand ( time(NULL) );
        test_CGBG_FF();
    }catch(E& e)
    {
        e.Print();
        return(1);
    }
    catch(char const* p)
    {
        cout << "Exception:" << p << endl;
        return(2);
    }
    return(0);
}//end of main

