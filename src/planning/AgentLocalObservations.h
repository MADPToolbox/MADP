/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTLOCALOBSERVATIONS_H_
#define _AGENTLOCALOBSERVATIONS_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentDecPOMDPDiscrete.h"

/** \brief AgentLocalObservations represents an agent that acts on local
 * observations. */
class AgentLocalObservations : 
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
    AgentLocalObservations(const PlanningUnitDecPOMDPDiscrete *pu, Index id) :
        AgentDecPOMDPDiscrete(pu,id){};

    /// Copy constructor.
    AgentLocalObservations(const AgentLocalObservations& a) :
        AgentDecPOMDPDiscrete(a){};

    /// Destructor.
    ~AgentLocalObservations(){};

    /** \brief Return an individual action index based on an
     *  individual observation index. */
    virtual Index Act(Index oI) = 0;
};


#endif /* !_AGENTLOCALOBSERVATIONS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
