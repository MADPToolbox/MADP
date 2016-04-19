/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
