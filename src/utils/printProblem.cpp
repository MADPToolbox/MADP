/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

// Very simple programs that only uses the Parser and Base libraries.

#include <iostream>
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

#include "argumentHandlers.h"

const char *argp_program_version = "printProblem";

// Program documentation
static char doc[] =
"printProblem - Print out the models of a problem. \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 2;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::modelOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try
    {

    ArgumentHandlers::Arguments args;

    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    DecPOMDPDiscreteInterface* dpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
    cout << dpomdp->SoftPrint() << endl;

    }
    catch(E& e){ e.Print(); }

    return(0);
}
