/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTFULLYOBSERVABLE_H_
#define _AGENTFULLYOBSERVABLE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentDecPOMDPDiscrete.h"
#include <vector>

/** \brief AgentFullyObservable represents an agent that receives the
 * true state, the joint observation and also the reward signal. */
class AgentFullyObservable : 
    virtual public AgentDecPOMDPDiscrete
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    /**a derived class should first call the AgentDecPOMDPDiscrete constructor.
     * Therefore, we do not need to pass along any arguments anymore here.
     */
    AgentFullyObservable(const PlanningUnitDecPOMDPDiscrete *pu, Index id) :
        AgentDecPOMDPDiscrete(pu,id){};

    /// Copy constructor.
    AgentFullyObservable(const AgentFullyObservable& a) :
        AgentDecPOMDPDiscrete(a){};

    /// Destructor.
    ~AgentFullyObservable(){};

    /** \brief Return an individual action based on state, last joint
     * observation and reward. */
    virtual Index Act(Index sI, Index joI, double reward) = 0;
};


#endif /* !_AGENTFULLYOBSERVABLE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
