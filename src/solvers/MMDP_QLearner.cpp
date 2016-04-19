/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include <fstream>
#include <vector>

#include "DecPOMDPDiscrete.h"
#include "Timing.h"
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

const char *argp_program_version = "MMDP_QLearner";

// Program documentation
static char doc[] =
"MMDP_QLearner - loads an MMDP problem, and learns a policy online with Q-learning. \
This only works for infinite horizon, so you have the specify a discount < 1. \
\vFor more information please consult the MADP documentation. \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    /* should probably use RL_child instead of simulation_child the future,
       we are /learning/ during simulations. */
    ArgumentHandlers::simulation_child,
    { 0 }
};
#include "argumentHandlersPostChild.h"

double runSimulations(const PlanningUnitDecPOMDPDiscrete *pu,
                      const SimulationDecPOMDPDiscrete &sim,
                      bool verbose)
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

    if(verbose)
    {
        QTable q = static_cast<AgentQLearner*>(agents[0])->GetQTable();
        row_t row   = q.GetRow(0);
        cout << setprecision(7)
             << "Here's first row (state 0) from Q-learning result: \n" << row << endl;
    }

    double avgReward = result.GetAvgReward();
    for(Index i=0; i < pu->GetNrAgents(); i++)
        delete agents[i];

    return avgReward;
}

bool file_exists(const string& fileName)
{
    ofstream file(fileName.c_str());
    return file.good();
}

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    try
    {
        Timing Time;
        cout << "Instantiating the problem..."<<endl;
        DecPOMDPDiscreteInterface* decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
        cout << "...done."<<endl;

        // consider it an infinite-horizon problem by passing MAXHORIZON
        PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(MAXHORIZON, decpomdp);

        //set up output files
        string filename="/dev/null", timingsFilename="/dev/null";
        if(!args.dryrun)
        {
            stringstream ss;
            ss  << directories::MADPGetResultsFilename("QLearner", *decpomdp, args)
                << "_h" << args.horizon;
            filename=ss.str();
            timingsFilename=filename + "_Timings";
            if(!file_exists(filename))
            {
                cout << "QLearner: could not open " << filename <<endl;
                cout << "Results will not be stored to disk." <<endl;
                args.dryrun = true;
            }
        }

        ///---VI--------------------------------------------
        if(args.verbose)
        {
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
        }
        ///---end VI----------------------------------------------

        int nrRuns = args.nrRuns; //defaults to 1000, see argumentHandlers.h
        int seed = args.randomSeed; //defaults to 42
        SimulationDecPOMDPDiscrete sim(*np, nrRuns, seed);

        //write intermediate simulation results to file
        if(!args.dryrun)
            sim.SaveIntermediateResults(filename);

        double r;
        cout << "Running q learning with nrRuns: "
             << nrRuns << " and seed: " << seed <<endl;
        Time.Start("Learn");
        r = runSimulations(np, sim, args.verbose);
        Time.Stop("Learn");
        cout << "Avg reward of "<< nrRuns << " simulations: " << r << endl << endl;

        //write q learning timing information to file
        if(!args.dryrun)
            Time.Save(timingsFilename);
    }
    catch(E& e){ e.Print(); }

    return 0;
}
