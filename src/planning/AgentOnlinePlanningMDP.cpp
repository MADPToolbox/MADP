/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

#include <iostream>

#include "AgentOnlinePlanningMDP.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "OnlineMDPPlanner.h"

using namespace std;

#define DEBUG_AgentOnlinePlanningMDP 0

AgentOnlinePlanningMDP::AgentOnlinePlanningMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                               OnlineMDPPlanner *planner) :
    AgentDecPOMDPDiscrete(pu, id),
    AgentFullyObservable(pu, id),
    _m_planner(planner),
    _m_t(0)
{
}

AgentOnlinePlanningMDP::AgentOnlinePlanningMDP(const AgentOnlinePlanningMDP& a) :
    AgentDecPOMDPDiscrete(a),
    AgentFullyObservable(a),
    _m_planner(a._m_planner),
    _m_t(a._m_t)
{
}

//Destructor
AgentOnlinePlanningMDP::~AgentOnlinePlanningMDP()
{
}

Index AgentOnlinePlanningMDP::Act(Index sI, Index joI, double reward)
{
    if(_m_planner) // only plan if online MDP planner is available
                   // Pure learners will have this set to NULL
    {
        Index jaInew;
        if(GetIndex()==0) // we only need to search for one agent, to
                          // avoid duplicate work
            jaInew=_m_planner->SearchForAction(sI, joI);
        else
            jaInew=_m_planner->GetLastActionChosen();

#if DEBUG_AgentOnlinePlanningMDP
    cout << GetIndex() << ": s " << sI
         << " ja " << jaInew << endl;
#endif

        vector<Index> aIs=GetPU()->JointToIndividualActionIndices(jaInew);

        _m_t++;

        return(aIs[GetIndex()]);
    }

    // else may still be a learner, leave Act implementation to subclass
    throw(E("AgentOnlinePlanningMDP::Act error, no planning module available"));
    return INT_MAX;
}

void AgentOnlinePlanningMDP::ResetEpisode()
{
    _m_t=0;
    if(_m_planner)
        _m_planner->Reset();
}
