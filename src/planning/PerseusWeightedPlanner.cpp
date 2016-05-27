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
