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
#include "directories.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "JointPolicyPureVector.h"
#include "NullPlanner.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "evaluateJointPolicyPureVector";

// Program documentation
static char doc[] =
"evaluateJointPolicyPureVector - Evaluates a JointPolicyPureVector  \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::jpolIndex_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::simulation_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    int h=args.horizon;
    LIndex index=args.jpolIndex;

    try {

    DecPOMDPDiscreteInterface* decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
    if(args.discount!=-1)
        decpomdp->SetDiscount(args.discount);
    PlanningUnitMADPDiscreteParameters params;
#if 0
    params.SetComputeAll(true);
    params.SetComputeJointBeliefs(false);
#else
    params.SetComputeAll(false);
#endif
    NullPlanner np(params,h,decpomdp);
    JointPolicyPureVector jp(&np);
    jp.SetIndex(index);

    SimulationDecPOMDPDiscrete sim(np,args);
    SimulationResult result;

    result=sim.RunSimulations(&jp);

    cout << "Reward h " << h << " reward: " << result.GetAvgReward() << endl;

    }
    catch(E& e){ e.Print(); }

    return(0);
}
