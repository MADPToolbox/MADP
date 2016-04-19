/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
    Index ActFirstStage()
    {return Act();}
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
