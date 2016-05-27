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
