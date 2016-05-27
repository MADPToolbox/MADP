/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "PerseusConstrainedPOMDPPlanner.h"
#include "BeliefValue.h"
#include <float.h>
#include <fstream>
#include <limits.h>

using namespace std;

#define DEBUG_PerseusConstrainedPOMDPPlanner 0
#define PRUNE_QFUNCTIONS 0
//Default constructor
PerseusConstrainedPOMDPPlanner::PerseusConstrainedPOMDPPlanner(const 
                                                               PlanningUnitDecPOMDPDiscrete* pu,
                                                               const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorConstrainedPOMDP(pu, params.falseNegativeObs),
    PerseusQFunctionPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}
PerseusConstrainedPOMDPPlanner::PerseusConstrainedPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                                                               const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorConstrainedPOMDP(pu, params.falseNegativeObs),
    PerseusQFunctionPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

//Destructor
PerseusConstrainedPOMDPPlanner::~PerseusConstrainedPOMDPPlanner()
{
}

QFunctionsDiscrete
PerseusConstrainedPOMDPPlanner::BackupStage(const BeliefSet &S,
                                            const QFunctionsDiscrete &Q) const
{
    vector<vector<double> > VB;
    vector<double>  VBalpha; 
    
    size_t nrB=S.size(),
           nrNotImproved=nrB,
           nrA=GetPU()->GetNrJointActions(),
           nrS=GetPU()->GetNrStates(),
           k;
    vector<bool> stillNeedToBeImproved(nrB,true);
    vector<vector<bool> > logicTable;
    vector<unsigned int> numberOfImprovements(nrB,0);

    QFunctionsDiscrete Q1(Q.size());
    ValueFunctionPOMDPDiscrete V1;
    AlphaVector alpha(nrS);

#if PRUNE_QFUNCTIONS

    for(Index a = 0; a < nrA; ++a)
    {
      Q1[a] = AlphaVectorPlanning::Prune(Q[a]);
    }

    V1 = AlphaVectorPlanning::QFunctionsToValueFunction(Q1);
#else
    V1 = AlphaVectorPlanning::QFunctionsToValueFunction(Q);
#endif

    for(Index a = 0; a < nrA; ++a)
    {
        Q1[a].clear();
        if(Q[a].empty())
        {
            stringstream estream;
            estream << "Empty Q-function for action " << a;
            throw E(estream.str());
        }
        VB.push_back(BeliefValue::GetValues(S,Q[a]));
        logicTable.push_back(stillNeedToBeImproved);
    }

    GaoVectorSet Gao=BackupStageLeadIn(V1);

    VectorSet Gab;

    while(nrNotImproved > 0)
    {
        int nrImprovedByGab=0;

        // sample a belief index from the number of not improved beliefs
        k=SampleNotImprovedBeliefIndex(stillNeedToBeImproved,nrNotImproved);

        Gab = BeliefBackupQ(*S[k],Gao,V1);
        for(size_t a = 0; a < nrA; a++)
        {
            alpha.SetAction(a);
            for(size_t s=0;s < nrS;s++)
              alpha.SetValue(Gab(a,s),s);

            // check whether the alphas improve the value of S[k]
            double x=S[k]->InnerProduct(alpha.GetValues());
            // if not, get copy from old value function
            if(x<VB[a][k])
            {
                alpha=BeliefValue::GetMaximizingVector(S,k,Q[a]);
#if DEBUG_PerseusConstrainedPOMDPPlanner
                cout << "Getting n-1 vector for " << k << endl;
#endif
            }
            
            // add alphas to Q1
            if(!VectorIsInValueFunction(alpha,Q1[a]))
                Q1[a].push_back(alpha);

            for(size_t b=0;b < nrB;b++)
            {
                if(!stillNeedToBeImproved[b])
                    continue;

                alpha=BeliefValue::GetMaximizingVector(S,b,Q1[a]);
                x=S[b]->InnerProduct(alpha.GetValues());

                if(logicTable[a][b] && x >= VB[a][b] - Globals::REWARD_PRECISION)
                {
                    logicTable[a][b] = false;
                    numberOfImprovements[b]++;
                }
                
                if(stillNeedToBeImproved[b] && numberOfImprovements[b] >= nrA)
                {
                    stillNeedToBeImproved[b]=false;
                    nrNotImproved--;
                    nrImprovedByGab++;
                }
            }
        }
        // update which beliefs have been improved

        if(GetVerbose() >= 0)
          cout << "Added vector for " << k << " ("
               << " improved " << nrImprovedByGab << " nrNotImproved: " << nrNotImproved << ")" << endl;
    }

    BackupStageLeadOut(Gao);

    return(Q1);
}
