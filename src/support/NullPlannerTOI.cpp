/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
