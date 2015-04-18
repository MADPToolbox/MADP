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

/* Only include this header file once. */
#ifndef _AGENTPOMDP_H_
#define _AGENTPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentSharedObservations.h"
#include "QFunctionJAOHInterface.h"
#include "QAV.h"
#include "PerseusPOMDPPlanner.h"

class JointBeliefInterface;

/**AgentPOMDP represents an agent which POMDP-based policy. */
class AgentPOMDP : public AgentSharedObservations
{
private:    
    
    QAV<PerseusPOMDPPlanner> *_m_QPOMDP;

    size_t _m_t;

    JointBeliefInterface *_m_jb;
    
    Index _m_prevJaI;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentPOMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
               QAV<PerseusPOMDPPlanner> *QPOMDP);

    /// Copy constructor.
    AgentPOMDP(const AgentPOMDP& a);

    /// Destructor.
    ~AgentPOMDP();

    Index Act(Index joI);

    void ResetEpisode();

};


#endif /* !_AGENTPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
