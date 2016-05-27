/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PerseusWeightedPlanner.h"


using namespace std;

#define DEBUG_PerseusWeightedPlanner 0

//Default constructor
PerseusWeightedPlanner::
PerseusWeightedPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                       const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu,params.weight),
    PerseusBGPOMDPPlanner(pu)
{
     AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}


PerseusWeightedPlanner::
PerseusWeightedPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

PerseusWeightedPlanner::
PerseusWeightedPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                       const QAVParameters& params) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu,params.weight),
    PerseusBGPOMDPPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}


PerseusWeightedPlanner::
PerseusWeightedPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorWeighted(pu),
    PerseusBGPOMDPPlanner(pu)
{
    AlphaVectorPlanning::Initialize();
    Perseus::Initialize();
}

//Destructor
PerseusWeightedPlanner::~PerseusWeightedPlanner()
{
}

AlphaVector
PerseusWeightedPlanner::BeliefBackup(const JointBeliefInterface &b,
                                     Index a,
                                     const GaoVectorSet &G,
                                     const QFunctionsDiscrete& Q) const
{
     return(AlphaVectorWeighted::BeliefBackup(b,a,G,Q));
}
