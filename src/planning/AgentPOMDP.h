/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
