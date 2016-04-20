/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTTOIFULLYOBSERVABLESYNCED_H_
#define _AGENTTOIFULLYOBSERVABLESYNCED_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentDecPOMDPDiscrete.h"
#include <vector>



/**AgentTOIFullyObservableSynced represents an agent that receives the
 * true state, the joint observation and also the reward
 * signal. Additionally, before acting, each agent's Sync() method
 * gets called. */
class AgentTOIFullyObservableSynced : public AgentDecPOMDPDiscrete
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentTOIFullyObservableSynced(const PlanningUnitDecPOMDPDiscrete *pu,
                               Index id) :
        AgentDecPOMDPDiscrete(pu,id){};

    /// Copy constructor.
    AgentTOIFullyObservableSynced(const AgentTOIFullyObservableSynced& a) :
        AgentDecPOMDPDiscrete(a){};

    /// Destructor.
    virtual ~AgentTOIFullyObservableSynced(){};
   
    virtual void Sync(const std::vector<Index> &sIs,
                      const std::vector<Index> &oIs, double reward) = 0;

    virtual Index Act(const std::vector<Index> &sIs,
                       const std::vector<Index> &oIs, double reward) = 0;
};


#endif /* !_AGENTTOIFULLYOBSERVABLESYNCED_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
