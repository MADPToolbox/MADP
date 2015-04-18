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
 * Julian Kooij 
 *
 * For contact information please see the included AUTHORS file.
 */

#include <time.h>
#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include "DICEPSPlanner.h"
#include "Timing.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "DICEPSPlanner";

// Program documentation
static char doc[] =
"DICEPSPlanner - runs the DICEPSPlanner  \
\v\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/DICEPS.                    | \
+-------------------------------------------------------------------------+ \
";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 5;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::CE_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    // parse the command line arguments
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    int restarts = args.nrCERestarts;

    if(args.verbose >= 0)
        cout << "DICE: direct CE Policy Search"<<endl;

    srand(time(0));

    int horizon = args.horizon;
    if(args.verbose >= 1)
        cout << "Horizon = " << horizon << endl;

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
    params.SetComputeJointActionHistories(false);
    params.SetComputeIndividualActionObservationHistories(false);
    params.SetComputeIndividualActionHistories(false);
    //params.SetComputeIndividualObservationHistories(false);
    // joint observations histories are needed for
    // efficient computation of joint actions
    params.SetComputeJointObservationHistories(true);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    DICEPSPlanner* planner;
    planner = new DICEPSPlanner (params, &decpomdp,
        horizon,
        //CE params
        args.nrCERestarts,
        args.nrCEIterations,
        args.nrCESamples,
        args.nrCESamplesForUpdate, 
        args.CE_use_hard_threshold, //(gamma in CE papers)
        args.CE_alpha, //the learning rate
        args.nrCEEvalutionRuns //the number of evaluation runs
        , args.verbose
    );
    Time.Stop("PlanningUnit");
    if(args.verbose >= 0)
        cout << "DICEPSPlanner initialized" << endl;

    for(int restartI = 0; restartI < restarts; restartI++)
    {
        Time.Start("Plan");
        planner->Plan();
        Time.Stop("Plan");

        double V = planner->GetExpectedReward();
        cout << "value="<< V << endl;
        if(args.verbose >= 1)
        {
            planner->GetJointPolicyPureVector()->Print();
            cout <<  endl;
        }
    }
    /* clean up */

    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        Time.PrintSummary();
        planner->PrintTimersSummary();
    }

    delete planner;
    }
    catch(E& e){ e.Print(); }
}
