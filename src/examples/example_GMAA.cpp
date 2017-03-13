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

#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>

#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "GMAA_MAAstar.h"
#include "GMAA_kGMAA.h"
#include "Timing.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace qheur;
using namespace GMAAtype;
using namespace BGIP_SolverType;
using namespace std;
using namespace ArgumentUtils;

/*
QFunctionJAOH* getQheuristic(GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaa,
                             Qheur_t Qheur)
{
    QFunctionJAOH* q = 0;
    switch(Qheur)
    {
    case(eQMDP):
        q = new QMDP(*gmaa,false);
        break;
    case(eQPOMDP):
        q = new QPOMDP(*gmaa);
        break;
    case(eQBG):
        q = new QBG(*gmaa);
        break;
    case(eQMDPc):
        q = new QMDP(*gmaa,true);
        break;
    }

    return(q);
}
*/

const char *argp_program_version = "GMAA";

// Program documentation
static char doc[] =
"GMAA - runs example version Generalized MAAStar planners \
\vFor more information please consult the MADP documentation. \
\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/GMAA.                      | \
+-------------------------------------------------------------------------+ \
";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::gmaa_child,
    ArgumentHandlers::qheur_child,
    { 0 }
};


#include "argumentHandlersPostChild.h"

GeneralizedMAAStarPlannerForDecPOMDPDiscrete* GetGMAAInstance(
    DecPOMDPDiscreteInterface* decpomdp,
    ArgumentHandlers::Arguments &args,
    const PlanningUnitMADPDiscreteParameters &params,
    const string &filename,
    BGIP_SolverCreatorInterface * bgipsc_p
    );

void GetBGIPSolverCreatorInstances(
    ArgumentHandlers::Arguments &args,
    BGIP_SolverCreatorInterface * & bgipsc_p
    );



int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    Qheur_t Qheur=args.qheur;
   
try 
{

    Timing Time;    
    Time.Start("Overall");
    srand(time(0));

    cout << "Instantiating the problem..."<<endl;
    DecPOMDPDiscreteInterface* decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
    cout << "...done."<<endl;

    GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaa=0;
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete * gmaaFirstInstance = 0;
    QFunctionJAOHInterface *q=0;

    string filename="",timingsFilename="", jpolFilename="";
    ofstream of;
    ofstream of_jpol;

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointActionObservationHistories(false);
    params.SetComputeJointObservationHistories(false);
    params.SetComputeJointActionHistories(false);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    params.SetComputeIndividualActionObservationHistories(false);
    params.SetComputeIndividualActionHistories(false);
    params.SetComputeIndividualObservationHistories(false);


    if(args.verbose >= 1)
        cout << "Instantiating the planning unit..."<<endl;
    //GMAA makes use of CBG solvers, here we create the object that will create the 
    //CBG solvers inside GMAA:
    BGIP_SolverCreatorInterface * bgipsc_p = 0;
    GetBGIPSolverCreatorInstances(args,bgipsc_p);
    if(bgipsc_p && args.verbose >= 1)    
        cout << "BGIP_SolverCreatorInterface instance: " << bgipsc_p->SoftPrint() << endl;

    // We need to make sure that the first GMAA instance exists until
    // the end of the program, as the Q-heuristic will use
    // functionality from it. It's a circular dependence...
    gmaaFirstInstance=GetGMAAInstance(decpomdp,args,params,"", bgipsc_p);
    gmaa=gmaaFirstInstance;
    if(args.verbose >= 0)
        cout << "GMAA Planner initialized" << endl;

    if(args.verbose >= 1)
        cout << "Computing the Q heuristic ("<<Qheur<<")..."<<endl;
    q=GetQheuristicFromArgs(gmaa,args);
    q->Compute();    
    if(args.verbose >= 0)
        cout << "Q heuristic computed" << endl;

    gmaa->SetQHeuristic(q);
    for(int restartI = 0; restartI < args.nrRestarts; restartI++)
    {
        gmaa->Plan();
        double V = gmaa->GetExpectedReward();
        cout << "\nvalue="<< V << endl;
        //JointPolicyDiscretePure* found_jpol =  gmaa->GetJointPolicyDiscretePure();
        boost::shared_ptr<JointPolicyDiscretePure> found_jpol =  
            gmaa->GetJointPolicyDiscretePure();
        if(args.verbose >= 1)
            cout << found_jpol->SoftPrint();
    }

    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        Time.PrintSummary();
        gmaa->PrintTimersSummary();
    }
    
    delete q;
    delete gmaa;
    delete decpomdp;
}
catch(E& e){ 
    e.Print(); 
    exit(-1); 
}
}



#include "BGIP_SolverType.h"
#include "BGIP_SolverCreator_AM.h"
#include "BGIP_SolverCreator_BFS.h" 
#include "BGIP_SolverCreator_CE.h"
#include "BGIP_SolverCreator_MP.h"
#include "BGIP_SolverCreator_BnB.h" 
#include "BGIP_SolverCreator_Random.h" 



void GetBGIPSolverCreatorInstances(
    ArgumentHandlers::Arguments &args,
    BGIP_SolverCreatorInterface * & bgipsc_p
    )
{
    if(args.gmaa!=MAAstarClassic) // Classic uses a built-in solver
    {
        // for MAAstar we need to make sure we keep all the solutions,
        // so set k to INT_MAX
        // XXX, Frans: what happens if nrSolutions > INT_MAX ?
        if(args.gmaa==MAAstar)
            args.k=INT_MAX;
        switch(args.bgsolver)
        {
        case BFS:
            if(args.useBGclustering)
                bgipsc_p = 
                    new BGIP_SolverCreator_BFS<JointPolicyPureVectorForClusteredBG>(
                        args.verbose, args.k);
            else
                bgipsc_p = new BGIP_SolverCreator_BFS<JointPolicyPureVector>(
                    args.verbose, args.k);
            break;
        case AM:
            if(args.useBGclustering)
                bgipsc_p = 
                    new BGIP_SolverCreator_AM<JointPolicyPureVectorForClusteredBG>(
                        args.nrAMRestarts, args.verbose, args.k);
            else
                bgipsc_p = new BGIP_SolverCreator_AM<JointPolicyPureVector>(
                    args.nrAMRestarts, args.verbose, args.k);
            break;

        case CE:       
            if(args.useBGclustering)
                throw(E("BGIP_SolverCE does not work yet with clustered policies"));
            else
                bgipsc_p = new BGIP_SolverCreator_CE(
                    args.nrCERestarts,
                    args.nrCEIterations,
                    args.nrCESamples,
                    args.nrCESamplesForUpdate,
                    args.CE_use_hard_threshold,
                    args.CE_alpha);
            break;

        case MaxPlus:
            if(args.useBGclustering)
                bgipsc_p =
                    new BGIP_SolverCreator_MP<JointPolicyPureVectorForClusteredBG> (
                    args.maxplus_maxiter, args.maxplus_updateT, 
                    args.maxplus_verbose, args.maxplus_damping, 
                    args.k,//<- nr solutions to return by BG solver
                    args.maxplus_nrRestarts
                    );
            else
                bgipsc_p = new BGIP_SolverCreator_MP<JointPolicyPureVector> (
                    args.maxplus_maxiter, args.maxplus_updateT, 
                    args.maxplus_verbose, args.maxplus_damping, 
                    args.k,//<- nr solutions to return by BG solver
                    args.maxplus_nrRestarts
                    );
            break;
        case BnB:
            if(args.useBGclustering)
                bgipsc_p =
                    new BGIP_SolverCreator_BnB<JointPolicyPureVectorForClusteredBG>
                    (args.verbose -2 , args.k, args.BnB_keepAll,
                     args.BnBJointTypeOrdering,
                     args.BnB_consistentCompleteInformationHeur);
            else
                bgipsc_p = new BGIP_SolverCreator_BnB<JointPolicyPureVector>
                    (args.verbose -2 , args.k, args.BnB_keepAll,
                     args.BnBJointTypeOrdering,
                     args.BnB_consistentCompleteInformationHeur);
            break;
            
        case CGBG_MaxPlus:
            throw E("CGBG max plus only works with factored Dec-POMDPs (use GMAAF instead)");
            break;

        case Random:
            
            if(args.useBGclustering)
                throw E("Random not implemented for clustered GMAA");
            else
                bgipsc_p = new BGIP_SolverCreator_Random
                    (args.verbose, args.k);
            break;

        default:
            throw E("BGIP_Solver is not handled");
            break;
        }

        if(bgipsc_p == 0 && bgipsc_p == 0)
            throw(E("No BGIP Solver Creator instantiated"));
    }
}

#include "QBG.h"
#include "QPOMDP.h"
#include "QMDP.h"
#include "qheur.h"
#include "gmaatype.h"

#include "GMAA_MAAstar.h"
#include "GMAA_MAAstarClassic.h"
#include "GMAA_MAAstarCluster.h"
#include "GMAA_kGMAA.h"
#include "GMAA_kGMAACluster.h"

GeneralizedMAAStarPlannerForDecPOMDPDiscrete* GetGMAAInstance(
    DecPOMDPDiscreteInterface* decpomdp,
    ArgumentHandlers::Arguments &args,
    const PlanningUnitMADPDiscreteParameters &params,
    const string &filename,
    BGIP_SolverCreatorInterface * bgipsc_p
    )
{
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaa=0;

    int verboseness = args.verbose;
    if(args.useBGclustering)
    {
        switch(args.gmaa)
        {
        case MAAstar:
            if(!bgipsc_p->IsExactSolver())
            {
                cout << "Error: MAAstar requires an exact BG solver, and " <<
                    bgipsc_p->SoftPrintBrief() << " is not" << endl;
                exit(1);
            }
            gmaa=new GMAA_MAAstarCluster(bgipsc_p, args.horizon, decpomdp, &params, verboseness );
            break;
        case FSPC:
            args.k=1; // fall through on purpose
        case kGMAA:
        {
            GMAA_kGMAACluster *gmaaCluster=
                new GMAA_kGMAACluster(
                    bgipsc_p, args.horizon, decpomdp, &params, args.k,
                    static_cast<BayesianGameWithClusterInfo::BGClusterAlgorithm>(
                        args.BGClusterAlgorithm)
                    );
            gmaaCluster->SetTresholdJB(args.thresholdJB);
            gmaaCluster->SetTresholdPjaoh(args.thresholdPjaoh);
            gmaa=gmaaCluster;
            break;
        }
        case MAAstarClassic:
            throw E("MAAstarClassic not implemented for clustered version");
            break;
        default:
            throw E("unrecognized GMAA type?!");
        }
    }
    else // regular, un-clustered BGs
    {
        switch(args.gmaa)
        {
        case MAAstar:
            if(!bgipsc_p->IsExactSolver())
            {
                cout << "Error: MAAstar requires an exact BG solver, and " <<
                    bgipsc_p->SoftPrintBrief() << " is not" << endl;
                exit(1);
            }
            gmaa=new GMAA_MAAstar(bgipsc_p, args.horizon, decpomdp, &params, verboseness );
            break;
        case FSPC:
            args.k=1; // fall through on purpose
        case kGMAA:
            gmaa=new GMAA_kGMAA(bgipsc_p, args.horizon, decpomdp, &params, args.k);
            break;
        case MAAstarClassic:
            gmaa=new GMAA_MAAstarClassic(args.horizon, decpomdp, &params, verboseness );
            break;
        default:
            throw E("unrecognized GMAA type?!");
        }
    }

    if(!args.dryrun)
    {
        if(args.saveAllBGs)
        {
            string bgFilename=filename + "_BG";
            gmaa->SetSaveAllBGs(bgFilename);
        }
    }
    
    gmaa->SetVerbose(args.verbose);
    if(args.GMAAdeadline)
        gmaa->SetDeadline(args.GMAAdeadline);

    return(gmaa);
}



