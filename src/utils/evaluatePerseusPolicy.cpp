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
#include "TOIDecPOMDPDiscrete.h"
#include "directories.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "AgentPOMDP.h"
#include "AgentBG.h"
#include "PerseusBackupType.h"
#include "AlphaVectorBG.h"
#include "Perseus.h"
#include "PerseusBGPlanner.h"
#include "BeliefValue.h"
#include "AlphaVectorPlanning.h"
#include "JointBelief.h"
#include <float.h>
#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

using namespace std;

const char *argp_program_version = "evaluatePerseusPolicy";

// Program documentation
static char doc[] =
"evaluatePerseusPolicy - simulate Perseus value functions \
\v";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::perseusbackup_child,
    ArgumentHandlers::simulation_child,
    ArgumentHandlers::eventPomdp_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try {

    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    QAVParameters qavParams=Perseus::ProcessArguments(args);

    // this is necessary for running Perseus on TOI models
    args.cache_flat_models=true;

    DecPOMDPDiscreteInterface* decpomdp;
    decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);

    stringstream valueFunction;
    valueFunction << directories::MADPGetResultsFilename("POMDP",*decpomdp,args)
                  << "Perseus"
                  << Perseus::BackupTypeToString(qavParams) 
                  << "ValueFunction_h";
    if(args.infiniteHorizon)
        valueFunction << MAXHORIZON;
    else
        valueFunction << args.horizon;

    size_t horizon;
    if(!args.infiniteHorizon)
    {
        horizon=args.horizon;
//         if(qavParams.stationary && !args.isTOI)
//             decpomdp->ConvertFiniteToInfiniteHorizon(horizon);
    }
    else
        horizon=MAXHORIZON;
   

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);

    params.SetEventObservability(decpomdp->GetEventObservability());

    PlanningUnitDecPOMDPDiscrete *np=new NullPlanner(horizon,decpomdp, &params);

    SimulationDecPOMDPDiscrete sim(*np,args);
    SimulationResult result;

    switch(args.backup)
    {
    case POMDP:
    {
        vector<AgentSharedObservations*> agents;
        AgentSharedObservations *agent1;
        QAV<PerseusPOMDPPlanner> *Qpomdp=
            new QAV<PerseusPOMDPPlanner>(np,valueFunction.str());
        for(unsigned int i=0;i!=decpomdp->GetNrAgents();++i)
        {
            agent1=new AgentPOMDP(np,0,Qpomdp);
            agent1->SetIndex(i);
            agents.push_back(agent1);
        }
        result=sim.RunSimulations(agents);
        break;
    }
    case BG:
    {
        vector<AgentDelayedSharedObservations*> agents;
        QAV<PerseusBGPlanner> *Qbg=
            new QAV<PerseusBGPlanner>(np,valueFunction.str());
        AgentDelayedSharedObservations *agent1;
        for(unsigned int i=0;i!=decpomdp->GetNrAgents();++i)
        {
            agent1=new AgentBG(np,0,Qbg);
            agent1->SetIndex(i);
            agents.push_back(agent1);
        }
        result=sim.RunSimulations(agents);
        break;
    }
    default:
        throw("PerseusBackupType is unknown");
    }

    double Vjb0;
    if(qavParams.stationary)
        Vjb0=
            BeliefValue::GetValue(JointBelief(*decpomdp->GetISD()),
                                  AlphaVectorPlanning::
                                  ImportValueFunction(valueFunction.str()));
    else
    {
        Vjb0=
            BeliefValue::GetValue(JointBelief(*decpomdp->GetISD()),
                                  AlphaVectorPlanning::
                                  ImportValueFunction(valueFunction.str(),
                                                      horizon, decpomdp->
                                                      GetNrJointActions(),
                                                      decpomdp->
                                                      GetNrStates()),
                                  0);
    }
    if(args.verbose < 0)
        cout << result.GetAvgReward() << " " << Vjb0 << endl;
    else
    {
        cout << "Empirical control quality (h " << horizon << " backup "
             << Perseus::BackupTypeToString(qavParams)
             << " V(jb0) " << Vjb0 << ") = " << result.GetAvgReward() << endl;
    }

    }
    catch(E& e){ e.Print(); }

    return(0);
}
