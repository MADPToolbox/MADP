/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_
#define _AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentTOIFullyObservableSynced.h"
#include <vector>

/**AgentTOIFullyObservableSyncedSpecialReward represents an
 * AgentTOIFullyObservableSynced. After acting, each agent's
 * GetSpecialReward() method gets called, and those reward are
 * incorporated in the overall reward. */
class AgentTOIFullyObservableSyncedSpecialReward : 
    public AgentTOIFullyObservableSynced
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentTOIFullyObservableSyncedSpecialReward(
        const PlanningUnitDecPOMDPDiscrete *pu,
        Index id) :
        AgentTOIFullyObservableSynced(pu,id){};

    /// Copy constructor.
    AgentTOIFullyObservableSyncedSpecialReward(
        const AgentTOIFullyObservableSyncedSpecialReward& a) :
        AgentTOIFullyObservableSynced(a){};

    /// Destructor.
    ~AgentTOIFullyObservableSyncedSpecialReward(){};
   
    virtual double GetSpecialReward() = 0;

};


#endif /* !_AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
