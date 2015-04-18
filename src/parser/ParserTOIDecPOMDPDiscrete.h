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
#ifndef _PARSERTOIDECPOMDPDISCRETE_H_
#define _PARSERTOIDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "TOIDecPOMDPDiscrete.h"
#include "DecPOMDPDiscrete.h"
#include "ParserInterface.h"



/**\brief ParserTOIDecPOMDPDiscrete is a parser for
 * TOIDecPOMDPDiscrete. */
class ParserTOIDecPOMDPDiscrete :
    public ParserInterface
{
private:    
    
    TOIDecPOMDPDiscrete *_m_problem;

    void ParseBase();
    void ParseAgent(Index id);

    virtual void StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
                               Index id);

protected:

    void ParseRewards(size_t nrAgents,
                      const std::vector<size_t> &nrStates,
                      const std::vector<size_t> &nrActions);
    virtual void ParseRewards();
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserTOIDecPOMDPDiscrete(TOIDecPOMDPDiscrete
                              *problem);

    virtual ~ParserTOIDecPOMDPDiscrete(){};

    void Parse();

};


#endif /* !_PARSERTOIDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
