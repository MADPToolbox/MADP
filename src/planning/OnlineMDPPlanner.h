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
