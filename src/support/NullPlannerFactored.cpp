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

#include "NullPlannerFactored.h"

NullPlannerFactored::NullPlannerFactored(FactoredDecPOMDPDiscreteInterface* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);

}
    
NullPlannerFactored::NullPlannerFactored(size_t horizon, FactoredDecPOMDPDiscreteInterface* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);
    SetHorizon(horizon);
}

NullPlannerFactored::NullPlannerFactored(const PlanningUnitMADPDiscreteParameters &params,
                         size_t horizon, FactoredDecPOMDPDiscreteInterface* p) :
    PlanningUnitFactoredDecPOMDPDiscrete(params,horizon,p)
{
}

void NullPlannerFactored::Plan()
{
    throw(E("NullPlannerFactored::Plan() should not be called"));
}

double NullPlannerFactored::GetExpectedReward() const
{
    throw(E("NullPlannerFactored::GetExpectedReward() should not be called"));
    return(0);
}

boost::shared_ptr<JointPolicy> NullPlannerFactored::GetJointPolicy()
{
    throw(E("NullPlannerFactored::GetJointPolicy() should not be called"));
    return(boost::shared_ptr<JointPolicy>());
}
