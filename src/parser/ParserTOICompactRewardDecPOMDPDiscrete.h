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
