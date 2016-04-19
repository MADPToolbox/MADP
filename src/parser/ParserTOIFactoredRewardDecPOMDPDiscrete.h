/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARSERTOIFACTOREDREWARDDECPOMDPDISCRETE_H_
#define _PARSERTOIFACTOREDREWARDDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "ParserTOIDecPOMDPDiscrete.h"
#include "DecPOMDPDiscrete.h"



/**\brief ParserTOIFactoredRewardDecPOMDPDiscrete is a parser for
 * TransitionObservationIndependentFactoredRewardDecPOMDPDiscrete. */
class ParserTOIFactoredRewardDecPOMDPDiscrete :
    public ParserTOIDecPOMDPDiscrete
{
private:    
    
    TOIFactoredRewardDecPOMDPDiscrete *_m_problem;

    void StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
                       Index id);

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserTOIFactoredRewardDecPOMDPDiscrete(
        TOIFactoredRewardDecPOMDPDiscrete
        *problem);

};


#endif /* !_PARSERTOIFACTOREDREWARDDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
