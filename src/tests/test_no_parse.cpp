/**\file test_no_parse.cpp
 *
 * Authors:
 * Frans Oliehoek <faolieho@science.uva.nl>
 * Matthijs Spaan <mtjspaan@isr.ist.utl.pt>
 *
 * Copyright 2008 Universiteit van Amsterdam, Instituto Superior Tecnico
 *
 * This file is part of MultiAgentDecisionProcess.
 *
 * MultiAgentDecisionProcess is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MultiAgentDecisionProcess is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiAgentDecisionProcess.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * $Id$
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


