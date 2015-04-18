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
