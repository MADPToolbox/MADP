/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTQMDP_H_
#define _AGENTQMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentSharedObservations.h"
#include "QFunctionJAOHInterface.h"
#include "JointBeliefSparse.h"
#include "QTable.h"



/**AgentQMDP represents an agent which uses a QMDP-based policy. */
class AgentQMDP : public AgentSharedObservations
{
private:    
    
    QTable _m_Q;

    size_t _m_t;

    JointBeliefSparse _m_jb;
    
    Index _m_prevJaI;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentQMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
              const QTable &Q);

    /// Copy constructor.
    AgentQMDP(const AgentQMDP& a);

    /// Destructor.
    ~AgentQMDP();

    Index Act(Index joI);

    void ResetEpisode();

};


#endif /* !_AGENTQMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
