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
