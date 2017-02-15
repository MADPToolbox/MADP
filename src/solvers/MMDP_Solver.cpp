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
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "config.h"
#include <iostream>
#include <fstream>

#include "DecPOMDPDiscrete.h"
#include "Timing.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "directories.h"

#include "MDPValueIteration.h"
#include "MDPPolicyIteration.h"
#if HAVE_CUDA_CUSOLVERDN_H
#include "MDPPolicyIterationGPU.h"
#endif

#include "QTable.h"
#include "AgentMDP.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"
using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "MMDP_Solver";

// Program documentation
static char doc[] =
"MMDP_Solver - loads an (MMDP) problem, runs value/policy iteration, and \
simulates the resulting joint policy using AgentMDP class. For \
more information please consult the MADP documentation.";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::simulation_child,
    ArgumentHandlers::mmdp_method_child,
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

        //construct MMDP_Solver
        PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(args.horizon, decpomdp);
        MDPSolver *mdpSolver=NULL;
        string methodName;
        switch (args.mmdp_method)
        {
        case ArgumentHandlers::ValueIteration:
            mdpSolver=new MDPValueIteration(*np);
            methodName="MMDP_Solver_VI";
            cout << "Running value iteration..."<<endl;
            break;
        case ArgumentHandlers::PolicyIteration:
            mdpSolver=new MDPPolicyIteration(*np);
            cout << "Running policy iteration..."<<endl;
            methodName="MMDP_Solver_PI";
            break;
        case ArgumentHandlers::PolicyIterationGPU:
#if HAVE_CUDA_CUSOLVERDN_H
            mdpSolver=new MDPPolicyIterationGPU(*np);
            cout << "Running GPU supported policy iteration..."<<endl;
            methodName="MMDP_Solver_PIgpu";
#else
            cerr<<"No 'MDPPolicyIterationGPU' available. Install cuda, reconfigure and recompile."<<endl;
            exit(1);
#endif
            break;
        default:
            cerr<<"Unkown mmdp_method"<<endl;
            exit(1);
        }

        //set up output files
        string filename="/dev/null", timingsFilename="/dev/null";
        if(!args.dryrun)
        {
            stringstream ss;
            ss  << directories::MADPGetResultsFilename(methodName, *decpomdp, args)
                << "_h" << args.horizon;
            filename=ss.str();
            timingsFilename=filename + "_Timings";
            if(!file_exists(filename))
            {
                cout << "MMDP_Solver: could not open " << filename <<endl;
                cout << "Results will not be stored to disk." <<endl;
                args.dryrun = true;
            }
        }

        //start planning
        Time.Start("Plan");
        mdpSolver->Plan(); // calls PlanSlow() on MDPPolicyIteration and MDPPolicyIterationGPU objects

        Time.Stop("Plan");
        cout << "...done."<<endl;
        QTable q = mdpSolver->GetQTable(0); //<- infinite horizon, so get 1 value function of stage 0

        int nrRuns = args.nrRuns; //defaults to 1000, see argumentHandlers.h
        int seed = args.randomSeed; //defaults to 42
        cout << "Simulating policy with nrRuns: " << nrRuns << " and seed: " << seed <<endl;
        SimulationDecPOMDPDiscrete sim(*np, nrRuns, seed);

        //write intermediate simulation results to file
        if(!args.dryrun)
            sim.SaveIntermediateResults(filename);

        vector<double> avgRewards;
        double r = runOneSimulation(q, np, sim);
        cout << "...done"<<endl;

        avgRewards.push_back(r);
        cout << "Avg rewards: " << SoftPrintVector(avgRewards) << endl;

        //write MDPSOLVER timing information to file
        if(!args.dryrun)
            Time.Save(timingsFilename);
    }
    catch(E& e){ e.Print(); }

    return(0);
}
