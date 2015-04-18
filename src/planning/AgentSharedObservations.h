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
#ifndef _AGENTSHAREDOBSERVATIONS_H_
#define _AGENTSHAREDOBSERVATIONS_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "AgentDecPOMDPDiscrete.h"

/** \brief AgentSharedObservations is represents an agent that
 * benefits from free communication, i.e., it can share all its
 * observations. */
class AgentSharedObservations : public AgentDecPOMDPDiscrete
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentSharedObservations(const PlanningUnitDecPOMDPDiscrete *pu, Index id) :
        AgentDecPOMDPDiscrete(pu,id){};

    /// Copy constructor.
    AgentSharedObservations(const AgentSharedObservations& a) :
        AgentDecPOMDPDiscrete(a){};

    /// Destructor.
    ~AgentSharedObservations(){};

    /** \brief Return an individual action index based on a joint
     *  observation index. */
    virtual Index Act(Index joI) = 0;

};


#endif /* !_AGENTSHAREDOBSERVATIONS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
