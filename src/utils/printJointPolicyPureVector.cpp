/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 3;
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
