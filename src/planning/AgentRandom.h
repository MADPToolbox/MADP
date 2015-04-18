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
#ifndef _AGENTRANDOM_H_
#define _AGENTRANDOM_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentFullyObservable.h"
#include "AgentLocalObservations.h"

/** \brief AgentRandom represents an agent which chooses action
 * uniformly at random. */
class AgentRandom : 
    public AgentFullyObservable 
    , public AgentLocalObservations 
{
private:    
    
public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentRandom(const PlanningUnitDecPOMDPDiscrete *pu, Index id);

    /// Copy constructor.
    AgentRandom(const AgentRandom& a);

    /// Destructor.
    ~AgentRandom();

    /// Returns an individual action uniformly at random.
    Index Act();
    Index Act(Index joI)
    {return Act();}
    Index Act(Index sI, Index joI, double reward) 
    {return Act();}

    void ResetEpisode();

};


#endif /* !_AGENTRANDOM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
