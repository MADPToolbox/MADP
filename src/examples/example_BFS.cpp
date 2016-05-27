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

#include <time.h>
#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>

#include "BruteForceSearchPlanner.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;


const char *argp_program_version = "BFS";

// Program documentation
static char doc[] =
"BFS - runs the example version of Brute Force SearchPlanner for Dec-POMDPs. \
\v\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/BFS.                       | \
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

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    int horizon=args.horizon;

    try {

    DecPOMDPDiscreteInterface & decpomdp = 
        * GetDecPOMDPDiscreteInterfaceFromArgs(args);

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);

    BruteForceSearchPlanner bfs(params,horizon,&decpomdp);

    if(args.verbose >= 0)
        cout << "BruteForceSearchPlanner initialized" << endl;

    bfs.Plan();
    double V = bfs.GetExpectedReward();
    cout << "\nvalue="<< V << endl;
    if(args.verbose >= 1)
        bfs.GetJointPolicyPureVector()->Print();

    }
    catch(E& e){ e.Print(); }
}
