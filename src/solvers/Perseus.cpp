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
#include "DecPOMDPDiscrete.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "PerseusPOMDPPlanner.h"
#include "PerseusConstrainedPOMDPPlanner.h"
#include "PerseusBGPlanner.h"
#include "PerseusBGNSPlanner.h"
#include "ParserTOIFactoredRewardDecPOMDPDiscrete.h"
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
"Perseus - runs Perseus planners \
\v";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::perseus_child,
    ArgumentHandlers::perseusbelief_child,
    ArgumentHandlers::perseusbackup_child,
    ArgumentHandlers::eventPomdp_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    bool errorOccurred=false;
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

    // this is necessary for running Perseus on TOI models
    args.cache_flat_models=true;

    DecPOMDPDiscreteInterface* decpomdp;
    decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);

    string resultsDir=directories::MADPGetResultsDir("POMDP",decpomdp);

    if(args.discount!=-1)
        decpomdp->SetDiscount(args.discount);

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    
    params.SetEventObservability(decpomdp->GetEventObservability());

    NullPlanner *np=new NullPlanner(horizon, decpomdp, &params);

    Perseus *P;
    PerseusStationary *PS=0;
    PerseusNonStationary *PNS=0;
    switch(args.backup)
    {
    case POMDP:
        PS=new PerseusPOMDPPlanner(np);
        break;
    case BG:
        PS=new PerseusBGPlanner(np,qavParams);
        break;
    case EVENT_POMDP:
    {
        PS=new PerseusConstrainedPOMDPPlanner(np,qavParams);
        break;
    }
    default:
        throw(E("PerseusBackupType is unknown"));
    }
    if(qavParams.stationary)
        P=PS;
    else
        P=PNS;

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
    cout << "Sampling " << args.nrBeliefs << " beliefs"; cout.flush();
    if(qavParams.stationary)
    {
        srand(42); ///this is necessary since Perseus::PlanLeadIn isn't sampling beliefs. 
        BeliefSet B=P->SampleBeliefs(args);

        PS->SetBeliefSet(B);

        if(args.saveBeliefs && !args.dryrun)
        {
            stringstream beliefFilename;
            beliefFilename << resultsDir << "/" << P->GetIdentification()
                           << "Beliefs"
                           << args.nrBeliefs << "_" << horizon << "_"
                           << args.resetAfter;
            if(args.useQMDPforSamplingBeliefs)
                beliefFilename << "_" << "QMDPsamplingPolicy"
                               << args.QMDPexploreProb;
            else
                beliefFilename << "_" << "randomSamplingPolicy";
            AlphaVectorPlanning::ExportBeliefSet(B,beliefFilename.str());
            if(args.verbose >= 0)
                cout << "Saved beliefs to " << beliefFilename.str() << endl;
        }
    }
    else
    {
        BeliefSetNonStationary B=
            P->SampleBeliefsNonStationary(args);

        if(args.verbose)
            B.Print();
        PNS->SetBeliefSet(B);
    }
    cout << "." << endl;

    if(args.savePOMDP && !args.dryrun)
    {
        stringstream POMDPfilename;
        POMDPfilename << resultsDir << "/" << decpomdp->GetUnixName();
        if(!args.infiniteHorizon)
            POMDPfilename << "_h" << horizon;
        POMDPfilename << ".POMDP";
        AlphaVectorPlanning::ExportPOMDPFile(POMDPfilename.str(),
                                             np->GetDPOMDPD());
    }

    if(args.saveIntermediateV && !args.dryrun)
        P->SetSaveIntermediateValueFunctions(true);
    if(args.saveTimings && !args.dryrun)
        P->SetSaveTimings(true);
    if(args.computeVectorForEachBelief)
        P->SetComputeVectorForEachBelief(true);
    if(args.dryrun)
        P->SetDryrun(true);
        
    P->Plan();

    if(args.verbose)
        P->PrintTimersSummary();
    }
    catch(E& e)
    { 
        e.Print();
        errorOccurred=true;
    }
    return errorOccurred;
}
