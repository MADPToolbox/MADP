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
 * Bas Terwijn
 *
 * For contact information please see the included AUTHORS file.
 */


#include <iostream>
#include <string>

#include "Action.h"
#include "ActionHistory.h"
#include "ProblemDecTiger.h"
#include "TransitionModelMapping.h"
#include "ObservationModelMapping.h"
#include "RewardModelMapping.h"
#include "JESPExhaustivePlanner.h"
#include "BruteForceSearchPlanner.h"


//#include "TestConstructActions.h"
#define BEEP 1
#define CATCH 1
#include "test_common_functions.h"


void TestMADPconstructors(const ArgumentHandlers::Arguments& args)
{
    const int NRAGENTS = 2;
    const int NRSTATES = 2;
    MultiAgentDecisionProcessDiscrete madpd(NRAGENTS, NRSTATES);
}

#include "argumentHandlers.h"
ArgumentHandlers::Arguments args;

// Name of program
const char *argp_program_version = "test_no_parse";

// Program documentation
static char doc[] =
"Some basic tests";

const struct argp_child childVector[] = {
    ArgumentHandlers::globalOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

#if CATCH
    try
    {
#endif       
        TestBeliefIterators(args);
        TestGMAA(args);
//TestNaming(args);
        TestMADPconstructors(args);
        TestJointBeliefSparse(args);
        TestPUMADP_jointToIndHistIndices(args);
        TestPUMADP_histOptions(args);
        TestGMAA(args);
        TestActionHistories(args);
        TestModelCreation(args);//test transitionmodel etc.
        TestValueFunction(args);
        TestBruteForceSearch(args);

#if CATCH
    }
    catch(E& e)
    {
        e.Print(); return(1);
    }
#endif        
}//end of main


