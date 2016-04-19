/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
