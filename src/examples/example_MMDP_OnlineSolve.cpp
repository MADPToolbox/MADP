/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

#include <iostream>
#include <vector>

#include "DecPOMDPDiscrete.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "directories.h"

#include "AgentQLearner.h"

#include "MDPValueIteration.h"
#include "QTable.h"
#include "AgentMDP.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"
using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "example_MMDP_OnlineSolve";

// Program documentation
static char doc[] =
"example_MMDP_OnlineSolve - loads an MMDP problem, and learns a policy online with Q-learning. \
This only works for infinite horizon, so you have the specify a discount < 1. \
\vFor more information please consult the MADP documentation. \
\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/MMDP_QLearner.             | \
+-------------------------------------------------------------------------+ \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 4;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    { 0 }
};
#include "argumentHandlersPostChild.h"

double runSimulations(const PlanningUnitDecPOMDPDiscrete *pu,
                      const SimulationDecPOMDPDiscrete &sim)
{
    // create 'template' agent from which others are created
    AgentQLearner agent(pu, 0, 0.0, 0.1, 0.9, pu->GetDiscount());

    AgentQLearner *newAgent;
    vector<AgentFullyObservable*> agents;

    for(Index i=0; i < pu->GetNrAgents(); i++)
    {
        newAgent=new AgentQLearner(agent);
        newAgent->SetIndex(i);
        agents.push_back(newAgent);
        
        // Only one agent, id 0, is learning the joint policy in this example.
        // Others merely look up their action component from that joint learner.
        newAgent->SetFirstAgent(static_cast<AgentQLearner*>(agents[0]));
    }
    SimulationResult result = sim.RunSimulations(agents);

    QTable q = static_cast<AgentQLearner*>(agents[0])->GetQTable();
    row_t row   = q.GetRow(0);
    cout << setprecision(7)
         << "Here's first row (state 0) from Q-learning result: \n" << row << endl;

    double avgReward = result.GetAvgReward();
    for(Index i=0; i < pu->GetNrAgents(); i++)
        delete agents[i];

    return avgReward;
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

        // consider it an infinite-horizon problem by passing MAXHORIZON
        PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(MAXHORIZON, decpomdp);

        ///---VI--------------------------------------------
        // only for reference, compute solution with value iteration
        cout << "Performing value iteration..."<<endl;
        MDPValueIteration vi(*np);
        vi.Plan();
        cout << "...done." <<endl;
        QTable q = vi.GetQTable(0); //<- infinite horizon, so get 1 value function of stage 0
        
        // output results of value iteration at a single state to allow
        // comparison with q learner results
        row_t row = q.GetRow(0);
        cout << setprecision(7)
             << "Here's first row (state 0) from VI result: \n" << row << endl;
        ///---end VI----------------------------------------------

        int nrRuns = 500;
        int seed = 42; //or: time(NULL)
        SimulationDecPOMDPDiscrete sim(*np, nrRuns, seed);

        double r;
        r = runSimulations(np, sim);
        cout << "Avg reward of "<< nrRuns << " simulations: " << r << endl << endl;

    }
    catch(E& e){ e.Print(); }

    return 0;
}
