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

#include <iostream>
#include "PerseusPOMDPPlanner.h"
#include "PerseusBGPlanner.h"
#include "TOIDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "directories.h"
#include "float.h"
#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "Perseus";

// Program documentation
static char doc[] =
"Perseus - runs example version Perseus planners \
\v\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/Perseus.                   | \
+-------------------------------------------------------------------------+ \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 7;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::perseus_child,
    ArgumentHandlers::perseusbelief_child,
    ArgumentHandlers::perseusbackup_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try
    {

    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    if(!args.infiniteHorizon)
    {
        cout << "Perseus requires an infinite-horizon setting (use --inf)" << endl;
        return(1);
    }

    size_t horizon;
    if(args.infiniteHorizon)
        horizon=MAXHORIZON;
    else
        horizon=args.horizon;
    
    QAVParameters qavParams=Perseus::ProcessArguments(args);

    DecPOMDPDiscreteInterface* decpomdp;
    decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);

    /*
    if(args.isTOI)
    {
        // necessary for running Perseus
        if(args.sparse)
        {
            cout << "Creating centralized sparse models"; cout.flush();
            dynamic_cast<TOIDecPOMDPDiscrete*>(decpomdp)->CreateCentralizedSparseModels();
            cout << "." << endl;
        }
        else
        {
            cout << "Creating centralized full models"; cout.flush();
            dynamic_cast<TOIDecPOMDPDiscrete*>(decpomdp)->CreateCentralizedFullModels();
            cout << "." << endl;
        }
    }*/

    if(args.discount!=-1)
        decpomdp->SetDiscount(args.discount);

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    NullPlanner *np=new NullPlanner(params,horizon,decpomdp);
    
    Perseus *P;
    PerseusStationary *PS=0;
    switch(args.backup)
    {
    case POMDP:
        PS=new PerseusPOMDPPlanner(np);
        break;
    case BG:
        PS=new PerseusBGPlanner(np,qavParams);
        break;
    default:
        throw(E("PerseusBackupType is unknown"));
    }
    P=PS;

    P->SetVerbose(args.verbose);
    if(args.minimumNrIterations)
        P->SetMinimumNumberOfIterations(args.minimumNrIterations);
    if(args.initializeWithZero)
        P->SetInitializeWithZero(true);
    if(args.initializeWithImmediateReward)
        P->SetInitializeWithImmediateReward(true);
    P->SetIdentification("Perseus" + Perseus::BackupTypeToString(qavParams));
    P->SetResultsFilename(directories::MADPGetResultsFilename("POMDP",
                                                              *decpomdp,args));
    if(args.verbose >= 0)
        cout << "Sampling " << args.nrBeliefs << " beliefs"; cout.flush();

    BeliefSet B=P->SampleBeliefs(args);

    PS->SetBeliefSet(B);

    if(args.verbose >= 0)
        cout << "." << endl;

    if(args.computeVectorForEachBelief)
        P->SetComputeVectorForEachBelief(true);
    P->SetDryrun(true);
    P->Plan();

    if(args.verbose >= 0)
        P->PrintTimersSummary();
    }
    catch(E& e){ e.Print(); }

    return(0);
}
