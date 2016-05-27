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

#include "PerseusBGPOMDPPlanner.h"
#include "BeliefValue.h"
#include <float.h>
#include <fstream>
#include <limits.h>

using namespace std;

#define DEBUG_PerseusBGPOMDPPlanner 0

//Default constructor
PerseusBGPOMDPPlanner::
PerseusBGPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    PerseusQFunctionPlanner(pu)
{
}
PerseusBGPOMDPPlanner::
PerseusBGPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    PerseusQFunctionPlanner(pu)
{
}

//Destructor
PerseusBGPOMDPPlanner::~PerseusBGPOMDPPlanner()
{
}

QFunctionsDiscrete
PerseusBGPOMDPPlanner::BackupStage(const BeliefSet &S,
                                   const QFunctionsDiscrete &Q) const
{
    if(_m_computeVectorForEachBelief)
        return(BackupStageAll(S,Q));
    else
        return(BackupStageSamplingAlt(S,Q));
}

QFunctionsDiscrete
PerseusBGPOMDPPlanner::BackupStageAll(const BeliefSet &S,
                                      const QFunctionsDiscrete &Q) const
{
    vector<double> VB=BeliefValue::GetValues(S,Q),
        VBalpha;

    int nrB=VB.size(),
        nrS=GetPU()->GetNrStates();
    QFunctionsDiscrete Q1(Q.size());
    AlphaVector alpha(nrS);

    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Q);
    GaoVectorSet Gao=BackupStageLeadIn(V);

    ValueFunctionPOMDPDiscrete Qalphas;

    for(int k=0;k<nrB;++k)
    {
        Qalphas.clear();
        for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
        {
            // backup the belief
            alpha=BeliefBackup(*S[k],a,Gao,Q);
           
            // add alpha to Q1
            if(!VectorIsInValueFunction(alpha,Q1[a]))
                Q1[a].push_back(alpha);
            Qalphas.push_back(alpha);
        }

        if(GetVerbose())
        {
            VBalpha=BeliefValue::GetValues(S,Qalphas);
            cout << "Added vectors for " << k << " (V " << VBalpha[k] 
                 << ")" << endl;
        }
    }

    BackupStageLeadOut(Gao);

    return(Q1);
}

QFunctionsDiscrete
PerseusBGPOMDPPlanner::BackupStageSamplingAlt(const BeliefSet &S,
                                              const QFunctionsDiscrete &Q) const
{
    vector<double> VB=BeliefValue::GetValues(S,Q),
        VBalpha;

    int nrB=VB.size(),
        nrNotImproved=nrB,
        nrS=GetPU()->GetNrStates(),
        k;
    vector<bool> stillNeedToBeImproved(nrB,true);
    QFunctionsDiscrete Q1(Q.size());
    AlphaVector alpha(nrS);

    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Q);
    GaoVectorSet Gao=BackupStageLeadIn(V);

    ValueFunctionPOMDPDiscrete Qalphas;

    k=-1;

    while(nrNotImproved!=0)
    {
        Qalphas.clear();
        for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
        {
            // sample a belief index from the number of not improved beliefs
            k=SampleNotImprovedBeliefIndex(stillNeedToBeImproved,
                                           nrNotImproved);

            // backup the belief
            alpha=BeliefBackup(*S[k],a,Gao,Q);

            // check whether alpha improves the value of S[k]
            double x=S[k]->InnerProduct(alpha.GetValues());
            // if not, get copy from old value function
            if(x<VB[k]) //QB[a][k])//VB[k])
            {
#if DEBUG_PerseusBGPOMDPPlanner
                cout << "Getting n-1 vector for action " << a << ", belief "
                     << k << " (" << x << " < " << VB[k] << ")" << endl;
#endif
                alpha=BeliefValue::GetMaximizingVector(S,k,Q[a]);
            }
            else
            {
                if(GetVerbose())
                    cout << "Added vector for action " << a << ", belief " 
                         << k << " (Q " << x << " >= " << VB[k] << ")"
                         << endl;
            }
            
            // add alpha to Q1
            if(!VectorIsInValueFunction(alpha,Q1[a]))
                Q1[a].push_back(alpha);
            Qalphas.push_back(alpha);
        }

        // update which beliefs have been improved
        VBalpha=BeliefValue::GetValues(S,Qalphas);
        int nrImprovedByAlpha=0;
        for(int b=0;b!=nrB;b++)
            if(stillNeedToBeImproved[b] && VBalpha[b]>=VB[b])
            {
                stillNeedToBeImproved[b]=false;
                nrNotImproved--;
                nrImprovedByAlpha++;
            }
        
        if(GetVerbose())
            cout << "Added vectors for " << k << " (V " << VBalpha[k] 
                 << " improved " << nrImprovedByAlpha << ")" << endl;
    }

    BackupStageLeadOut(Gao);

    return(Q1);
}
