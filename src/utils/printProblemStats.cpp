/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include "NullPlanner.h"
#include "directories.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "printProblemStats";

// Program documentation
static char doc[] =
"printProblemStats - Prints out some general information about a problem domain  \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 3;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::modelOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

string toEnotation(LIndex nr)
{
    stringstream ss;
    ss << nr;
    if(ss.str().size()>3)
    {
        stringstream ssE;
        ssE << ss.str().at(0) << "." << ss.str().at(1);
        ssE << ss.str().at(2);
        ssE << "e" << ss.str().size()-1;
        return(ssE.str());
    } 
    else
        return(ss.str());
}

int main(int argc, char **argv)
{
    try {

    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    DecPOMDPDiscreteInterface* decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);

    cout << "Problem " << decpomdp->GetUnixName() << " has:" << endl
         << decpomdp->GetNrAgents() << " agents" << endl
         << decpomdp->GetNrStates() << " states" << endl
         << decpomdp->GetNrJointActions() << " joint actions, per agent";
    for(Index i=0;i!=decpomdp->GetNrAgents();++i)
        cout << " " << decpomdp->GetNrActions(i);
    cout << endl
         << decpomdp->GetNrJointObservations() << " joint observations, per agent";
    for(Index i=0;i!=decpomdp->GetNrAgents();++i)
        cout << " " << decpomdp->GetNrObservations(i);
    cout << endl
         << "Statistics per horizon:" << endl;

    for(Index h=1;h<=(Index)args.horizon;++h)
    {
        NullPlanner np(h,decpomdp);
        cout << "h=" << h
             << " nrJOH=" << np.GetNrJointObservationHistories()
             << " nrJAOH=" << np.GetNrJointActionObservationHistories()
             << " nrJPOL=" << np.GetNrJointPolicies() << " (" << toEnotation(np.GetNrJointPolicies())
             << ")" << endl;
    }

    }
    catch(E& e){ e.Print(); }

    return(0);
}
