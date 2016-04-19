/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTDELAYEDSHAREDOBSERVATIONS_H_
#define _AGENTDELAYEDSHAREDOBSERVATIONS_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentDecPOMDPDiscrete.h"



/**AgentDelayedSharedObservations represents an agent that acts on local
 * observations and the shared observation at the previous time step. */
class AgentDelayedSharedObservations : public AgentDecPOMDPDiscrete
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentDelayedSharedObservations(const PlanningUnitDecPOMDPDiscrete *pu,
                                   Index id) :
        AgentDecPOMDPDiscrete(pu,id){};

    /// Copy constructor.
    AgentDelayedSharedObservations(const AgentDelayedSharedObservations& a) :
        AgentDecPOMDPDiscrete(a){};

    /// Destructor.
    ~AgentDelayedSharedObservations(){};
   
    virtual Index Act(Index oI, Index prevJoI) = 0;
};


#endif /* !_AGENTDELAYEDSHAREDOBSERVATIONS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
