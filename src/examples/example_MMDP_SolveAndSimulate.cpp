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

#include <iostream>

#include "DecPOMDPDiscrete.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "directories.h"

#include "MDPValueIteration.h"
#include "QTable.h"
#include "AgentMDP.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"
using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "example_MMDP_SolveAndSimulate";

// Program documentation
static char doc[] =
"example_MMDP_SolveAndSimulate - loads an (MMDP) problem, runs value iteration, and simulates the resulting joint policy using AgentMDP class. \
\vFor more information please consult the MADP documentation. \
\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/MMDP_VI.                   | \
+-------------------------------------------------------------------------+ \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 5;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::simulation_child,
    { 0 }
};
#include "argumentHandlersPostChild.h"


double runOneSimulation(  const QTable & q,
                          const PlanningUnitDecPOMDPDiscrete *np,
                          const SimulationDecPOMDPDiscrete &sim
                          )
{
    AgentMDP agent(np, 0, q);
    AgentFullyObservable *newAgent;
    vector<AgentFullyObservable*> agents;

    for(Index i=0; i < np->GetNrAgents(); i++)
    {
        newAgent=new AgentMDP(agent);
        newAgent->SetIndex(i);
        agents.push_back(newAgent);
    }
    SimulationResult result=sim.RunSimulations(agents);
    double avgReward=result.GetAvgReward();

    for(Index i=0; i < np->GetNrAgents(); i++)
        delete agents[i];

    return(avgReward);
}

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    try
    {
        cout << "Instantiating the problem..."<<endl;
        DecPOMDPDiscreteInterface* decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
        cout << "...done."<<endl;

        PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(args.horizon, decpomdp);
        MDPValueIteration vi(*np);
        vi.Plan();
        QTable q = vi.GetQTable(0); //<- infinite horizon, so get 1 value function of stage 0

        int nrRuns = args.nrRuns; //500;
        int seed = args.randomSeed;
        SimulationDecPOMDPDiscrete sim(*np, nrRuns, seed);
        vector<double> avgRewards;

        double r;
        r = runOneSimulation(q, np, sim );
        avgRewards.push_back(r);
        cout << "Avg rewards: " << SoftPrintVector(avgRewards) << endl;

    }
    catch(E& e){ e.Print(); }

    return(0);
}
