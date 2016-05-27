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

#include "PerseusBGNSPlanner.h"
#include "BeliefValue.h"
#include <float.h>
#include <fstream>
#include <limits.h>

using namespace std;

#define DEBUG_PerseusBGNSPlanner 0

//Default constructor
PerseusBGNSPlanner::PerseusBGNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                                       const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorBG(pu),
    PerseusNonStationaryQPlanner(pu),
    _m_backupType(params.bgBackupType)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusBGNSPlanner::PerseusBGNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorBG(pu),
    PerseusNonStationaryQPlanner(pu),
    _m_backupType(BGIP_SOLVER_EXHAUSTIVE)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusBGNSPlanner::PerseusBGNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                                       const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorBG(pu),
    PerseusNonStationaryQPlanner(pu),
    _m_backupType(params.bgBackupType)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusBGNSPlanner::PerseusBGNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorBG(pu),
    PerseusNonStationaryQPlanner(pu),
    _m_backupType(BGIP_SOLVER_EXHAUSTIVE)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

//Destructor
PerseusBGNSPlanner::~PerseusBGNSPlanner()
{
}

QFunctionsDiscrete
PerseusBGNSPlanner::BackupStage(const BeliefSet &S,
                                const QFunctionsDiscrete &Q) const
{
    throw(E("PerseusBGNSPlanner::BackupStage code needs to be updated to use a different last time step backup"));
    if(_m_computeVectorForEachBelief)
        return(BackupStageAll(S,Q));
    else
        return(BackupStageSampling(S,Q));
}

QFunctionsDiscrete
PerseusBGNSPlanner::BackupStageAll(const BeliefSet &S,
                                   const QFunctionsDiscrete &Q)
    const
{
    size_t nrS=GetPU()->GetNrStates(),
        nrJA=GetPU()->GetNrJointActions();
    QFunctionsDiscrete Q1(nrJA);
    AlphaVector alpha(nrS);

    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Q);
    // get the backprojected vectors
    GaoVectorSet Gao=BackupStageLeadIn(V);

    ValueFunctionPOMDPDiscrete Qalphas;

    // loop over all beliefs for this time step
    for(Index k=0;k<S.size();++k)
    {
        Qalphas.clear();
        for(Index a=0;a!=nrJA;++a)
        {
            // backup the belief
            alpha=AlphaVectorBG::BeliefBackup(*S[k],a,Gao,V,_m_backupType);
           
            // add alpha to Q1
            if(!VectorIsInValueFunction(alpha,Q1[a]))
                Q1[a].push_back(alpha);
            Qalphas.push_back(alpha);
        }
        
        if(GetVerbose())
        {
            vector<double> VBalpha=BeliefValue::GetValues(S,Qalphas);
            cout << "Added vectors for " << k
                 << " (V " << VBalpha[k] << ")" << endl;
        }
    }

    BackupStageLeadOut(Gao);

    return(Q1);
}

QFunctionsDiscrete
PerseusBGNSPlanner::BackupStageSampling(const BeliefSet &S,
                                        const QFunctionsDiscrete &Q) 
    const
{
    vector<double> VB=BeliefValue::GetValues(S,Q),
        VBalpha;
    vector<vector<double> > QB;
    for(unsigned int a=0;a!=Q.size();++a)
        QB.push_back(BeliefValue::GetValues(S,Q[a]));

    int nrB=VB.size(),
        nrNotImproved=nrB,
        nrS=GetPU()->GetNrStates(),
        k;
    double oldValue;
    QFunctionsDiscrete Q1(Q.size());
    vector<bool> stillNeedToBeImproved(nrB,true);
    AlphaVector alpha(nrS);

    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Q);
    GaoVectorSet Gao=BackupStageLeadIn(V);

    ValueFunctionPOMDPDiscrete Qalphas;

    while(nrNotImproved!=0)
    {
        // sample a belief index from the number of not improved beliefs
        k=SampleNotImprovedBeliefIndex(stillNeedToBeImproved,nrNotImproved);

        Qalphas.clear();
        for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
        {
            // backup the belief
            alpha=AlphaVectorBG::BeliefBackup(*S[k],a,Gao,V,_m_backupType);

            // check whether alpha improves the value of S[k]
            double x=S[k]->InnerProduct(alpha.GetValues());

            // if not, get copy from old value function
            oldValue=QB[a][k];//VB[k]
            if(x<oldValue)
            {
#if DEBUG_PerseusBGNSPlanner
                cout << "Getting n-1 vector for action " << a << ", belief "
                     << k << " (" << x << " < " <<oldValue << ")" << endl;
#endif
                alpha=BeliefValue::GetMaximizingVector(S,k,Q[a]);
            }
            else
            {
                if(GetVerbose())
                    cout << "Added vector for action " << a << ", belief " 
                         << k << " (Q " << x << " >= " << oldValue << ")"
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
