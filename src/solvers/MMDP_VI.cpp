/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include <fstream>

#include "DecPOMDPDiscrete.h"
#include "Timing.h"
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

const char *argp_program_version = "MMDP_VI";

// Program documentation
static char doc[] =
"MMDP_VI - loads an (MMDP) problem, runs value iteration, and simulates the resulting joint policy using AgentMDP class. \
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

        //set up output files
        string filename="/dev/null", timingsFilename="/dev/null";
        if(!args.dryrun)
        {
            stringstream ss;
            ss  << directories::MADPGetResultsFilename("VI", *decpomdp, args)
                << "_h" << args.horizon;
            filename=ss.str();
            timingsFilename=filename + "_Timings";
            if(!file_exists(filename))
            {
                cout << "VI: could not open " << filename <<endl;
                cout << "Results will not be stored to disk." <<endl;
                args.dryrun = true;
            }
        }

        //start VI
        PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(args.horizon, decpomdp);
        MDPValueIteration vi(*np);
        cout << "Running value iteration..."<<endl;
        Time.Start("Plan");
        vi.Plan();
        Time.Stop("Plan");
        cout << "...done."<<endl;
        QTable q = vi.GetQTable(0); //<- infinite horizon, so get 1 value function of stage 0

        int nrRuns = args.nrRuns; //defaults to 1000, see argumentHandlers.h
        int seed = args.randomSeed; //defaults to 42
        cout << "Simulating policy with nrRuns: "
             << nrRuns << " and seed: " << seed <<endl;
        SimulationDecPOMDPDiscrete sim(*np, nrRuns, seed);

        //write intermediate simulation results to file
        if(!args.dryrun)
            sim.SaveIntermediateResults(filename);

        vector<double> avgRewards;
        double r = runOneSimulation(q, np, sim);
        cout << "...done"<<endl;

        avgRewards.push_back(r);
        cout << "Avg rewards: " << SoftPrintVector(avgRewards) << endl;

        //write VI timing information to file
        if(!args.dryrun)
            Time.Save(timingsFilename);
    }
    catch(E& e){ e.Print(); }

    return(0);
}
