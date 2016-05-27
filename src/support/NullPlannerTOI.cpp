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

#include "NullPlannerTOI.h"

NullPlannerTOI::NullPlannerTOI(TOIDecPOMDPDiscrete* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);

}
    
NullPlannerTOI::NullPlannerTOI(size_t horizon, TOIDecPOMDPDiscrete* p)
{
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);

    SetParams(params);
    SetProblem(p);
    SetHorizon(horizon);
}

NullPlannerTOI::NullPlannerTOI(const PlanningUnitMADPDiscreteParameters &params,
                               size_t horizon,
                               TOIDecPOMDPDiscrete* p) :
    PlanningUnitTOIDecPOMDPDiscrete(params,horizon,p)
{
}

void NullPlannerTOI::Plan()
{
    throw(E("NullPlannerTOI::Plan() should not be called"));
}

double NullPlannerTOI::GetExpectedReward() const
{
    throw(E("NullPlannerTOI::GetExpectedReward() should not be called"));
    return(0);
}

boost::shared_ptr<JointPolicy> NullPlannerTOI::GetJointPolicy()
{
    throw(E("NullPlannerTOI::GetJointPolicy() should not be called"));
    return(boost::shared_ptr<JointPolicy>());
}
