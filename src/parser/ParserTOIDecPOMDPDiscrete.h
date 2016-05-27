/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
