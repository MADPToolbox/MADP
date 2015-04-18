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

#ifndef _AGENT_ONLINE_MDP_H_
#define _AGENT_ONLINE_MDP_H_ 1

#include <iostream>
#include "Globals.h"

#include "AgentFullyObservable.h"
#include "OnlineMDPPlanner.h"

/** \brief AgentOnlinePlanningMDP represents an agent with an online MDP policy.
 *
 * This class implements a basic online MDP planner given an online planning module.
 **/
class AgentOnlinePlanningMDP : public AgentFullyObservable
{
private:
    /// The planning module. Pure \a learners will have this set to NULL.
    OnlineMDPPlanner *_m_planner;

protected:
    /// The episode count.
    size_t _m_t;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentOnlinePlanningMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                   OnlineMDPPlanner *planner);

    /// Copy constructor.
    AgentOnlinePlanningMDP(const AgentOnlinePlanningMDP& a);

    /// Destructor.
    virtual ~AgentOnlinePlanningMDP();

    virtual Index Act(Index sI, Index joI, double reward);

    virtual void ResetEpisode();

};

#endif /* !_AGENT_ONLINE_MDP_H_ */
