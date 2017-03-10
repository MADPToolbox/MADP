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

#include <sys/times.h>
#include <iostream>
#include <fstream>

#include "ProblemDecTiger.h"
#include "ProblemDecTigerWithCreaks.h"
#include "ProblemFireFighting.h"
#include "ProblemAloha.h"
#include "ProblemFireFightingFactored.h"
#include "ProblemFireFightingGraph.h"
#include "ProblemFOBSFireFightingFactored.h"
#include "ProblemFOBSFireFightingGraph.h"

#include "GeneralizedMAAStarPlanner.h"
#include "GMAA_MAAstarClassic.h"
#include "QBG.h"
#include "QPOMDP.h"
#include "QMDP.h"

#include "OptimalValueDatabase.h"

#define BEEP 1

using namespace std;

namespace{
    enum Qheur_t {eQMDP, eQPOMDP, eQBG};

    //how often de we perform each experiment with unique settings?
    size_t uniqueExpRepeat = 1;
}

void TestGMAA_MAAstar(size_t h, DecPOMDPDiscreteInterface* pdt, Qheur_t qType)        
{
    cout << "------horizon="<<h<<" problem:"<<pdt->GetUnixName()<<endl;
    for(size_t repeat = 0; repeat < uniqueExpRepeat; repeat++)
    {
        PlanningUnitMADPDiscreteParameters params;
        params.SetComputeAll(true);
        GMAA_MAAstarClassic gmaa(h,pdt, &params);
        
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
        cout << "optimal value:"<< V<<endl;

        OptimalValueDatabase db(&gmaa);
        if(db.IsInDatabase())
        {
            if (db.IsOptimal(V))
                cout<<"OptimalValueDatabase: correct optimal value"<<endl;
            else
            {
                cout<<"OptimalValueDatabase: incorrect optimal value, should be '"<<db.GetOptimalValue()<<"'"<<endl;
                exit(1); // exit with error status
            }
        }
        else
        {
            cout<<"OptimalValueDatabase: not in database"<<endl;
            //db.SetOptimalValue(V); // add to database
        }
        delete q;            
    }
} 

class ProblemHorizonPair
{
public:
    ProblemHorizonPair(DecPOMDPDiscreteInterface* p,size_t minH,size_t maxH)
        : _problem(p),_minHorizon(minH),_maxHorizon(maxH)
    {}

    DecPOMDPDiscreteInterface* GetProblem() {return _problem;}
    size_t GetMinHorizon() {return _minHorizon;}
    size_t GetMaxHorizon() {return _maxHorizon;}

            
private:
    DecPOMDPDiscreteInterface* _problem;
    size_t _minHorizon,_maxHorizon;
};

int main()
{    
    vector<ProblemHorizonPair> problems;

    // DecPOMDPDiscrete
    problems.push_back(ProblemHorizonPair(new ProblemDecTiger(),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemDecTigerWithCreaks(),1,1));
    problems.push_back(ProblemHorizonPair(new ProblemFireFighting(2,3,3),2,2));

    // FactoredDecPOMDPDiscrete
    problems.push_back(ProblemHorizonPair(new ProblemAloha(ProblemAloha::OneIsland,
                                                           ProblemAloha::NoNewPacket),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemAloha(ProblemAloha::TwoIslands, 
                                                           ProblemAloha::NoNewPacket),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemAloha(ProblemAloha::ThreeIslandsInLine,
                                                           ProblemAloha::NoNewPacket),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemFireFightingFactored(2,3,3),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemFireFightingGraph(2,3),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemFireFightingGraph(3,3),2,2));
    problems.push_back(ProblemHorizonPair(new ProblemFOBSFireFightingFactored(2,3,3),1,1));
    problems.push_back(ProblemHorizonPair(new ProblemFOBSFireFightingGraph(2,3),1,1));
    

    for (vector<ProblemHorizonPair>::iterator it=problems.begin();it!=problems.end();++it)
    {
        for(size_t h = it->GetMinHorizon(); h <= it->GetMaxHorizon(); h++)
        {     
            cout << "---------------------------------------"<<endl;
            cout << "------gmaa MAA* QBG -------"<<endl;
            TestGMAA_MAAstar(h, it->GetProblem(), eQBG);        
            cout << "---------------------------------------"<<endl;
            cout << "------gmaa MAA* QPOMDP ----"<<endl;
            TestGMAA_MAAstar(h, it->GetProblem(), eQPOMDP);
            cout << "---------------------------------------"<<endl;
            cout << "------gmaa MAA* QMDP ------"<<endl;
            TestGMAA_MAAstar(h, it->GetProblem(), eQMDP);
        }
        delete it->GetProblem();
    }
    
    return 0;
}
