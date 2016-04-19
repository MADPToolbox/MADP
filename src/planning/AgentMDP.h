/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTMDP_H_
#define _AGENTMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentFullyObservable.h"
#include "QFunctionJAOHInterface.h"
#include "JointBeliefSparse.h"
#include "QTable.h"



/**AgentMDP represents an agent which uses a MDP-based policy.
 *
 * In particular, AgentMDP takes actions based upon states by looking 
 * up the maximizing action in the QTable (_m_Q) that this agent receives
 * upon construction. This is in contrast to other AgentFullyObservable types
 * that do some actual learning or planning during the simulation.
 * */
class AgentMDP : public AgentFullyObservable
{
private:    
    
    QTable _m_Q;

    size_t _m_t;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
             const QTable &Q);

    /// Copy constructor.
    AgentMDP(const AgentMDP& a);

    /// Destructor.
    ~AgentMDP();

    Index Act(Index sI, Index joI, double reward);

    void ResetEpisode();

};


#endif /* !_AGENTMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
