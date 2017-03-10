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
