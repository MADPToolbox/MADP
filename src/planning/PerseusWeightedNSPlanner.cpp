/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PerseusWeightedNSPlanner.h"
#include "CommModel.h"

using namespace std;

#define DEBUG_PerseusWeightedNSPlanner 0

//Default constructor
PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                         const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu,params.weight),
    PerseusBGPOMDPNSPlanner(pu)
{
    if(params.backup==25)
    {
        CommModel commModel(GetPU()->GetNrStates());
        // warning, won't work for TOI-DPOMDPs
        commModel.Load(directories::MADPGetProblemFilename(GetPU()->GetDPOMDPD()->
                                                           GetUnixName()),
                       params.commModel);
        vector<double> weights(GetPU()->GetNrStates());
        for(Index s=0;s!=GetPU()->GetNrStates();++s)
            weights[s]=commModel.Get(s,0);
        SetWeights(weights);
    }

    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                         const CommModel& comm) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPNSPlanner(pu)
{
    vector<double> weights(GetPU()->GetNrStates());
    for(Index s=0;s!=GetPU()->GetNrStates();++s)
        weights[s]=comm.Get(s,0);
    SetWeights(weights);
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPNSPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                         const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu,params.weight),
    PerseusBGPOMDPNSPlanner(pu)
{
    if(params.backup==25)
    {
        CommModel commModel(GetPU()->GetNrStates());
        // warning, won't work for TOI-DPOMDPs
        commModel.Load(directories::MADPGetProblemFilename(GetPU()->GetDPOMDPD()->
                                                           GetUnixName()),
                       params.commModel);
        vector<double> weights(GetPU()->GetNrStates());
        for(Index s=0;s!=GetPU()->GetNrStates();++s)
            weights[s]=commModel.Get(s,0);
        SetWeights(weights);
    }

    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                         const CommModel& comm) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPNSPlanner(pu)
{
    vector<double> weights(GetPU()->GetNrStates());
    for(Index s=0;s!=GetPU()->GetNrStates();++s)
        weights[s]=comm.Get(s,0);
    SetWeights(weights);
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedNSPlanner::
PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPNSPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

//Destructor
PerseusWeightedNSPlanner::~PerseusWeightedNSPlanner()
{
}

AlphaVector
PerseusWeightedNSPlanner::BeliefBackup(const JointBeliefInterface &b,
                                       Index a,
                                       const GaoVectorSet &G,
                                       const QFunctionsDiscrete& Q) const
{
     return(AlphaVectorWeighted::BeliefBackup(b,a,G,Q));
}
