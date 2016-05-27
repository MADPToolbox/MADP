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

#define CHECK_RESULT 0

#include <time.h>
#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include "JESPExhaustivePlanner.h"
#include "JESPDynamicProgrammingPlanner.h"
#include "Timing.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;


const char *argp_program_version = "JESP";

// Program documentation
static char doc[] =
"JESP - runs the JESP planner (example version)\
\v\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/JESP.                      | \
+-------------------------------------------------------------------------+ \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 5;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::JESP_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    int restarts = args.nrRestarts;

    srand(time(0));

    int horizon=args.horizon;

    try {
    //start timers
    Timing Time;    
    Time.Start("Overall");

    DecPOMDPDiscreteInterface & decpomdp = * GetDecPOMDPDiscreteInterfaceFromArgs(args);
    
    //Initialization of the planner with typical options for JESP:
    Time.Start("PlanningUnit");
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointActionObservationHistories(false);
    params.SetComputeJointObservationHistories(false);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    PlanningUnitDecPOMDPDiscrete* jesp = 0;
    if(args.jesp == JESPtype::JESPExhaustive)
    {
        jesp = new JESPExhaustivePlanner (params,horizon,&decpomdp);
        if(args.verbose >= 0)
            cout << "JESPExhaustivePlanner initialized" << endl;
    }
    else if(args.jesp == JESPtype::JESPDP)
    {
        jesp = new JESPDynamicProgrammingPlanner (params,horizon,&decpomdp);
        if(args.verbose >= 0)
            cout << "JESPDynamicProgrammingPlanner initialized" << endl;
    }
    Time.Stop("PlanningUnit");
    if(args.verbose >= 0)
        cout << "JESP Planner initialized" << endl;

    for(int restartI = 0; restartI < restarts; restartI++)
    {
        //start all timers:
        Time.Start("Plan");

        jesp->Plan();
        double V = jesp->GetExpectedReward();
        cout << "value="<< V << endl;
        if(args.verbose >= 1) 
        {
            jesp->GetJointPolicyPureVector()->Print();
            cout <<  endl;
        }

        Time.Stop("Plan");

#if CHECK_RESULT
        ValueFunctionDecPOMDPDiscrete vf(jesp, jesp->GetJointPolicyPureVector());
        double v = vf.CalculateV(true);
        cout << "Validated value (exact/approx):="<<v;
        SimulationDecPOMDPDiscrete sim(*jesp, 1000);
        SimulationResult simres = 
            sim.RunSimulations( jesp->GetJointPolicyPureVector() );
        v = simres.GetAvgReward();
        cout << " / "<<v <<endl;
#endif

    }

    delete jesp;
    
    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        Time.PrintSummary();
    }

    }
    catch(E& e){ e.Print(); }
}
