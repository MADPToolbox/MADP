/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
