/**\file test_GMAA_MAAstar.cpp
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

#include "ProblemDecTiger.h"
#include "GeneralizedMAAStarPlanner.h"
#include "GMAA_MAAstarClassic.h"
#include "QBG.h"
#include "QPOMDP.h"
#include "QMDP.h"

#define BEEP 1

using namespace std;

namespace{
    enum Qheur_t {eQMDP, eQPOMDP, eQBG};

    int cps = 100;//CLOCKS_PER_SEC;

    clock_t tot_ticks;
    clock_t tot_utime;
    //how often de we perform each experiment with unique settings?
    size_t uniqueExpRepeat = 1;


}

void TestGMAA_MAAstar(size_t h, DecPOMDPDiscrete& pdt, Qheur_t qType 
//        ofstream& of, ofstream& irof );
     );
void TestGMAA(size_t h, DecPOMDPDiscrete& pdt, Qheur_t qType);

int main()
{    
    ProblemDecTiger pdt;

    for(size_t h = 1; h < 4; h++)
    {

        cout << "---------------------------------------"<<endl;
        cout << "------gmaa MAA* QBG  -horizon="<<h<<"-------"<<endl;
        TestGMAA_MAAstar(h, pdt, eQBG);        
        cout << "---------------------------------------"<<endl;
        cout << "------gmaa MAA* QPOMDP  -horizon="<<h<<"----"<<endl;
        TestGMAA_MAAstar(h, pdt, eQPOMDP);
        cout << "---------------------------------------"<<endl;
        cout << "------gmaa MAA* QMDP  -horizon="<<h<<"------"<<endl;
        TestGMAA_MAAstar(h, pdt, eQMDP);
    }
    
    cout << "\n\nTotal ticks "<< tot_ticks << " clock ticks = " << 
        (double)(tot_ticks) / cps <<"s" << endl <<
        "Total utime = "<< tot_utime << " clock ticks = "<< 
        (double (tot_utime))/ cps << "s" << endl;
}

void TestGMAA_MAAstar(size_t h, DecPOMDPDiscrete& pdt, Qheur_t qType)        
{
    for(size_t repeat = 0; repeat < uniqueExpRepeat; repeat++)
    {

        PlanningUnitMADPDiscreteParameters params;
        params.SetComputeAll(true);
        GMAA_MAAstarClassic gmaa(h, &pdt, &params);
        //debugging:
 //       gmaa.SetVerbose(true);
        
        QFunctionJAOHInterface* q = 0;
        switch(qType)
        {
            case(eQMDP):
                q = new QMDP(&gmaa);
                break;
            case(eQPOMDP):
                q = new QPOMDP(&gmaa);
                break;
            case(eQBG):
                q = new QBG(&gmaa);
                break;
        }

        q->Compute();
        gmaa.SetQHeuristic(q);
        double V = -DBL_MAX;
        gmaa.Plan();
        V = gmaa.GetExpectedReward();
        cout << "\noptimal value="<< V;
        cout << endl;

        delete q;            

        cout << "nr of BG jpols evaluated="<<gmaa.GetNrEvaluatedJPolBGs()<<endl;
        cout << "Max nr of jpols in Pool="<<gmaa.GetMaxJPolPoolSize()<<endl;
            
    }//end for repeat

    return;
} 
