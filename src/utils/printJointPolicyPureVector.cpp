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
#include "argumentUtils.h"
#include "JointPolicyPureVector.h"
#include "NullPlanner.h"
#include "directories.h"

#include "argumentHandlers.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "printJointPolicyPureVector";

// Program documentation
static char doc[] =
"printJointPolicyPureVector - Prints out a JointPolicyPureVector  \
\v";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::jpolIndex_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try {

    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    DecPOMDPDiscreteInterface *dpomdp=GetDecPOMDPDiscreteInterfaceFromArgs(args);
    LIndex index=args.jpolIndex;
    NullPlanner *np=new NullPlanner(args.horizon,dpomdp);
    JointPolicyPureVector jp(np);

    if(index>=np->GetNrJointPolicies())
    {
        cout << "Error: index " << index << " is too high, there are only "
             << np->GetNrJointPolicies() << " joint policies." << endl;
        return(0);
    }
    jp.SetIndex(index);

    jp.Print();
    }
    catch(E& e){ e.Print(); }

    return(0);
}
