/**\file test_GMAA_MAA_ELSI.cpp
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
