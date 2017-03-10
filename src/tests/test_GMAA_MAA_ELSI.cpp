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


#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>

#include "MADPParser.h"
#include "DecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "ProblemFireFightingGraph.h"
#include "ProblemFireFightingFactored.h"
#include "ProblemDecTiger.h"
#include "GeneralizedMAAStarPlanner.h"
#include "GMAA_MAA_ELSI.h"
#include "GMAA_MAAstar.h"
#include "QBG.h"
#include "QPOMDP.h"
#include "QMDP.h"
#include "FactoredQFunctionStateJAOHInterface.h"
#include "FactoredQLastTimeStepOrQMDP.h"
#include "FactoredQLastTimeStepOrQPOMDP.h"
#include "FactoredQLastTimeStepOrQBG.h"
#define BEEP 1

using namespace std;

//TODO look at fireFightingExample.cpp to `load' the firefighting example.


namespace{
    enum Qheur_t {eQMDP, eQPOMDP, eQBG};

    int cps = 100;//CLOCKS_PER_SEC;

    clock_t tot_ticks;
    clock_t tot_utime;
    //how often de we perform each experiment with unique settings?
    size_t uniqueExpRepeat = 1;


}

void TestGMAA_MAA_ELSI(size_t h, FactoredDecPOMDPDiscreteInterface& fdpomdp, Qheur_t qType 
//        ofstream& of, ofstream& irof );
     );
//void TestGMAA(size_t h, FactoredDecPOMDPDiscrete& fdpomdp, Qheur_t qType);

int main()
{    
    try{
    //ProblemDecTiger fdpomdp;
//    ProblemFireFightingGraph_old ff("fireFightingGraph_3_4_3");
    //ProblemFireFightingGraph_old ff("fireFightingGraph_2_3_3");
//    MADPParser parser(&ff);

    ProblemFireFightingGraph ff(3,3);
    //ProblemFireFightingFactored ff(2,3,3);
    //ProblemFireFightingGraph ff(2,3);
    //ff.Initialize();
//    m.Print();
    cout << ff.GetUnixName() << endl;
        

    Index h1=3;
    for(size_t h = h1; h <= h1; h++)
    {

        cout << "---------------------------------------"<<endl;
        cout << "------gmaa MAA* QMDP -horizon="<<h<<"-------"<<endl;
        TestGMAA_MAA_ELSI(h, ff, eQMDP);        
    }
    
    cout << "\n\nTotal ticks "<< tot_ticks << " clock ticks = " << 
        (double)(tot_ticks) / cps <<"s" << endl <<
        "Total utime = "<< tot_utime << " clock ticks = "<< 
        (double (tot_utime))/ cps << "s" << endl;
    }catch(E e){
        e.Print();
    }
}

void TestGMAA_MAA_ELSI(size_t h, FactoredDecPOMDPDiscreteInterface& fdpomdp, Qheur_t qType)        
{
    for(size_t repeat = 0; repeat < uniqueExpRepeat; repeat++)
    {
        cout << "Starting initialization of planning unit..."<<endl;
        PlanningUnitMADPDiscreteParameters params;
        //params.SetComputeAll(false);
        params.SetComputeAllJointHistories(false);
        params.SetComputeJointBeliefs(false);
        GMAA_MAA_ELSI gmaa(h, &fdpomdp, &params);
        //debugging:
        gmaa.SetVerbose(true);
        
        FactoredQFunctionStateJAOHInterface* q=0;
        switch(qType)
        {
        case(eQMDP):
            q = new FactoredQLastTimeStepOrQMDP(&gmaa);
            break;
        case(eQPOMDP):
            q = new FactoredQLastTimeStepOrQPOMDP(&gmaa);
            break;
        case(eQBG):
            q = new FactoredQLastTimeStepOrQBG(&gmaa);
            break;
        }

        cout << "Starting computation of FactoredQLastTimeStepOr..."<<endl;
        q->Compute();
        cout << "Factored Q Last time step computed"<<endl;
        //gmaa.SetQHeuristic(q);
        gmaa.SetFactoredQHeuristic(q);
        gmaa.Plan();
        double V = gmaa.GetExpectedReward();
        cout << "\noptimal value="<< V;
        cout << endl;

        delete q;            

        cout << "nr of BG jpols evaluated="<<gmaa.GetNrEvaluatedJPolBGs()<<endl;
        cout << "Max nr of jpols in Pool="<<gmaa.GetMaxJPolPoolSize()<<endl;
            
    }//end for repeat

    return;
} 
