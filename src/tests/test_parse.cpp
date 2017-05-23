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
    double Vmax = -DBL_MAX;
    for(int i=0; i<1000; i++)
    {
        JESPe.Plan();
        Vsuboptimal = JESPe.GetExpectedReward();
        if(Vsuboptimal > Vmax)
        {
            Vmax = Vsuboptimal;
            //cout << "\n>>>new maximum!"<<endl;
        }
    }
    cout << "Max. Expected value: " << Vmax<<"\n\n"<<endl;
    cout << "\n---------------------------------------"<<endl;
//    cout << "press enter to continue...";
//    cin.get();


    
}

