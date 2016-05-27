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

#include "NullPlanner.h"

NullPlanner::NullPlanner(DecPOMDPDiscreteInterface* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);

}
    
NullPlanner::NullPlanner(size_t horizon, DecPOMDPDiscreteInterface* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);
    SetHorizon(horizon);
}

NullPlanner::NullPlanner(const PlanningUnitMADPDiscreteParameters &params,
                         size_t horizon, DecPOMDPDiscreteInterface* p) :
    PlanningUnitDecPOMDPDiscrete(params,horizon,p)
{
}

void NullPlanner::Plan()
{
    throw(E("NullPlanner::Plan() should not be called"));
}

double NullPlanner::GetExpectedReward() const
{
    throw(E("NullPlanner::GetExpectedReward() should not be called"));
    return(0);
}

boost::shared_ptr<JointPolicy> NullPlanner::GetJointPolicy()
{
    throw(E("NullPlanner::GetJointPolicy() should not be called"));
    return(boost::shared_ptr<JointPolicy>());
}
