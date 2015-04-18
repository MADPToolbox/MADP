/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include "PerseusPOMDPPlanner.h"
#include "BeliefValue.h"
#include <float.h>
#include <fstream>
#include <limits.h>

using namespace std;

#define DEBUG_PerseusPOMDPPlanner 0

//Default constructor
PerseusPOMDPPlanner::PerseusPOMDPPlanner(const 
                                         PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    PerseusStationary(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}
PerseusPOMDPPlanner::PerseusPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    PerseusStationary(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

//Destructor
PerseusPOMDPPlanner::~PerseusPOMDPPlanner()
{
}

void PerseusPOMDPPlanner::Plan()
{
    PlanLeadIn();

    ValueFunctionPOMDPDiscrete V0,V1;
    vector<double> VB,VBnew;
    
    // get initial value function
    V1=GetInitialValueFunction();
    VB=BeliefValue::GetValues(*_m_beliefs,V1);

    int iter=0;                      
    bool done=false;
    while(!done)
    {
        // print out some info
        PlanStartOfIteration(iter,VB,V1);

        // the real thing: compute the next stage value function
        V0=V1;
        V1=BackupStage(*_m_beliefs,V0);

        // kind of hack to fix a problem of an persisting initial
        // alpha vector (which has the INT_MAX action)
        if(V1.size()>1)
        {
            ValueFunctionPOMDPDiscrete::iterator it=V1.begin();
            while(it!=V1.end())
            {
                if(it->GetAction()==INT_MAX)
                {
                    V1.erase(it);
                    break;
                }
                it++;
            }
        }

        // compute the maximum difference in the values for all
        // beliefs: for the convergence test
        VBnew=BeliefValue::GetValues(*_m_beliefs,V1);
        
        // test for convergence
        if(CheckConvergence(VB,VBnew,iter))
            done=true;

        VB=VBnew;
        iter++;
        
        PlanEndOfIteration(V1);
    }

    PlanLeadOut();
}

ValueFunctionPOMDPDiscrete
PerseusPOMDPPlanner::BackupStage(const BeliefSet &S,
                                 const ValueFunctionPOMDPDiscrete &V)
{
    vector<double> VB=BeliefValue::GetValues(S,V),
        VBalpha;
    int nrB=VB.size(),
        nrNotImproved=nrB,
        nrS=GetPU()->GetNrStates(),
        k;
    vector<bool> stillNeedToBeImproved(nrB,true);
    ValueFunctionPOMDPDiscrete V1;
    AlphaVector alpha(nrS);

    GaoVectorSet Gao=BackupStageLeadIn(V);

    if(_m_computeVectorForEachBelief)
        k=-1;

    while(nrNotImproved!=0)
    {
        if(_m_computeVectorForEachBelief)
            k++;
        else // sample a belief index from the number of not improved beliefs
            k=SampleNotImprovedBeliefIndex(stillNeedToBeImproved,nrNotImproved);

        alpha=BeliefBackup(*S[k],Gao);

        if(!_m_computeVectorForEachBelief)
        {
            // check whether alpha improves the value of S[k]
            double x=S[k]->InnerProduct(alpha.GetValues());
            // if not, get copy from old value function
            if(x<VB[k])
            {
                alpha=BeliefValue::GetMaximizingVector(S,k,V);
                
#if DEBUG_PerseusPOMDPPlanner
                cout << "Getting n-1 vector for " << k << endl;
#endif
            }
        }

        // add alpha to V1
        V1.push_back(alpha);

        if(_m_computeVectorForEachBelief)
            nrNotImproved--;
        else
        {
            // update which beliefs have been improved
            VBalpha=BeliefValue::GetValues(S,alpha);
            int nrImprovedByAlpha=0;
            for(int b=0;b!=nrB;b++)
                if(stillNeedToBeImproved[b] && VBalpha[b]>=VB[b])
                {
                    stillNeedToBeImproved[b]=false;
                    nrNotImproved--;
                    nrImprovedByAlpha++;
                }

            if(GetVerbose() >= 0)
                cout << "Added vector for " << k << " (V " << VBalpha[k] 
                     << " improved " << nrImprovedByAlpha << ")" << endl;
#if DEBUG_PerseusPOMDPPlanner
            alpha.Print();
            if(nrImprovedByAlpha==0)
                abort();
#endif
        }
    }

    BackupStageLeadOut(Gao);

    return(V1);
}
