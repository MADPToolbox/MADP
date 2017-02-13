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
#include <fstream>
#include "DecPOMDPDiscrete.h"
#include "directories.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationFactoredDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "NullPlannerFactored.h"
#include "AgentRandom.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "evaluateRandomPolicy";

// Program documentation
static char doc[] =
"evaluateRandomPolicy - Evaluates a random policy  \
\v";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::simulation_child,
    ArgumentHandlers::modelOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try {

    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    DecPOMDPDiscreteInterface* decpomdp=0;
    FactoredDecPOMDPDiscreteInterface* fdecpomdp=0;
    bool factoredModel=false;

    decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
    fdecpomdp = dynamic_cast<FactoredDecPOMDPDiscreteInterface*>(decpomdp);
    if(fdecpomdp)
        factoredModel=true;

    // no need to compute all histories
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    NullPlanner *np=0;
    NullPlannerFactored *npFactored=0;
    if(factoredModel)
        npFactored=new NullPlannerFactored(args.horizon,fdecpomdp,&params);
    else
        np=new NullPlanner(args.horizon,decpomdp,&params);

    ofstream of;
    if(!args.dryrun)
    {
        // Create results dir if it doesn't exist already
        directories::MADPCreateResultsDir("evaluateRandomPolicy",*decpomdp);
        stringstream ss;
        ss  << directories::MADPGetResultsFilename("evaluateRandomPolicy",
                                                   *decpomdp,args)
            << "h" << args.horizon << "_nrRuns" << args.nrRuns;
        string filename=ss.str();
        of.open(filename.c_str());
        if(!of)
        {
            cerr << "evaluateRandomPolicy: could not open "
                 << filename << endl;
            return(1);
        }
        
        cout << "Computing " << filename << endl;
    }

    SimulationResult result;
    if(factoredModel)
    {
        SimulationFactoredDecPOMDPDiscrete sim(*npFactored,args);
        result=sim.RunSimulationsRandomActions();
    }
    else
    {
        SimulationDecPOMDPDiscrete sim(*np,args);
        vector<AgentFullyObservable*> agents;
        for(unsigned int i=0;i!=np->GetNrAgents();++i)
        {
            AgentFullyObservable *agent1;
            AgentRandom agent(np,0);
            agent1=new AgentRandom(agent);
            agent1->SetIndex(i);
            agents.push_back(agent1);
        }
        result=sim.RunSimulations(agents);
    }
    cout << "evaluateRandomPolicy sampled value: "
         << result.GetAvgReward() << endl;


    if(!args.dryrun)
        of << result.GetAvgReward() << endl;


    }
    catch(E& e){ e.Print(); }

    return(0);
}
