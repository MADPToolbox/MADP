/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARSERTOICOMPACTREWARDDECPOMDPDISCRETE_H_
#define _PARSERTOICOMPACTREWARDDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "TOICompactRewardDecPOMDPDiscrete.h"
#include "ParserTOIDecPOMDPDiscrete.h"
#include "DecPOMDPDiscrete.h"



/**\brief ParserTOICompactRewardDecPOMDPDiscrete is a parser for
 * TOICompactRewardDecPOMDPDiscrete. */
class ParserTOICompactRewardDecPOMDPDiscrete :
    public ParserTOIDecPOMDPDiscrete
{
private:    
    
    TOICompactRewardDecPOMDPDiscrete *_m_problem;

    void StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
                       Index id);

    using ParserTOIDecPOMDPDiscrete::ParseRewards;
    void ParseRewards();

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserTOICompactRewardDecPOMDPDiscrete(
        TOICompactRewardDecPOMDPDiscrete
        *problem);

};


#endif /* !_PARSERTOICOMPACTREWARDDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
