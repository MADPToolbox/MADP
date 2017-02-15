/**\file test_parse.cpp
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
#include "IndexTools.h"
#include "ProblemDecTiger.h"
#include "TransitionModelMapping.h"
#include "ObservationModelMapping.h"
#include "RewardModelMapping.h"
#include "JESPExhaustivePlanner.h"
#include "BruteForceSearchPlanner.h"
//#include "TestConstructActions.h"
#include "MADPParser.h"
#define BEEP 1
#include "test_common_functions.h"

void TestParsing(const ArgumentHandlers::Arguments& args);

using namespace std;

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
    try
    {
        //TestNaming(args);
        TestParsing(args);
        TestModelCreation(args);
        TestValueFunction(args);
        TestBruteForceSearch(args);
    }
    catch(E& e){ e.Print();}
    
}//end of main

void TestParsing(const ArgumentHandlers::Arguments& args)
{
    cout << "----------------------------------"<<endl;
    cout << "-         TestParsing()          -"<<endl;
    cout << "----------------------------------"<<endl;

    DecPOMDPDiscrete test("Test parse problem", "parses the test.dpomdp file ", "../parser/test.dpomdp");
    
    try{
        cout << " testing \"test.MultiAgentDecisionProcess::Print()\"... :\n";
        test.MultiAgentDecisionProcess::Print();
        cout << endl << "<" <<endl;
        cout << " testing \"test.GetProblemFile()\"... :\n";
        cout << test.GetProblemFile();
        cout << endl << "<" <<endl;
        
        test.Print();
    }
    catch (E& e){e.Print();}

    try{
        cout << "\n>>PARSE TEST: test" << endl;
        MADPParser dpomdpd_parser_test (&test);
    }
    catch (E& e){e.Print();}
    try{test.Print();}
    catch (E& e){e.Print();}



    cout << "\n\n\n>>PARSE TEST: tiger" << endl;
    cout << "----------------------" << endl;    
    
    DecPOMDPDiscrete dectiger("DecTiger problem", "simple toy problem with 2 agents who have to jointly open the good (non-tiger) door", "../../problems/dectiger.dpomdp");
    try {dectiger.Print();}
    catch (E& e) {e.Print();}

    try{
        cout << "\n\n>>Start Parse:" << endl;
        MADPParser dpomdpd_parser (&dectiger);
    }
    catch (E& e){e.Print();}

    dectiger.Print();
    JESPExhaustivePlanner JESPe(2, &dectiger);
    //JESPe.SetProblem(&dectiger);
    //JESPe.SetHorizon(2);    

    cout << "---------------------------------------"<<endl;
    cout << "-JESPe.Print()---------------------"<<endl;
    cout << "---------------------------------------"<<endl;
    JESPe.Print();

    cout << "---------------------------------------"<<endl;
    cout << "-JESPe.Plan()-horizon=2----------------"<<endl;
    cout << "---------------------------------------"<<endl;

    if (args.testMode)
        JESPe.SetSeed(0);
    else
        JESPe.SetSeed(time(0));
    double Vsuboptimal = -DBL_MAX;
    double max = -DBL_MAX;
    for(int i=0; i<100; i++)
    {
        JESPe.Plan();
        Vsuboptimal = JESPe.GetExpectedReward();
        cout << ", "<< Vsuboptimal;
        if(Vsuboptimal > max)
        {
            max = Vsuboptimal;
            //cout << "\n>>>new maximum!"<<endl;
        }
    }
    cout << "\n---------------------------------------"<<endl;
//    cout << "Max. Expected value: " << max<<"\n\n"<<endl;
//    cout << "press enter to continue...";
//    cin.get();


    
}

