/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef _ONLINE_MDP_PLANNER_H_
#define _ONLINE_MDP_PLANNER_H_ 1

#include "Globals.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief OnlineMDPPlanner provides an abstract base class for online MDP planners.
 **/
class OnlineMDPPlanner
{
private:

    const PlanningUnitDecPOMDPDiscrete *_m_pu;

    Index _m_lastActionChosen;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    OnlineMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu);

    /// Destructor.
    virtual ~OnlineMDPPlanner();

    virtual void Reset();

    virtual Index SearchForAction(Index sI, Index joI) = 0;

    Index GetLastActionChosen() const { return _m_lastActionChosen; }

    const PlanningUnitDecPOMDPDiscrete* GetPU() const { return(_m_pu); }
};

#endif /* !_ONLINE_MDP_PLANNER_H_ */
