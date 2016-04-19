/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARSERTOIDECMDPDISCRETE_H_
#define _PARSERTOIDECMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "TOIDecMDPDiscrete.h"
#include "ParserInterface.h"



/**\brief ParserTOIDecMDPDiscrete is a parser for
 * TOIDecMDPDiscrete. */
class ParserTOIDecMDPDiscrete :
    public ParserInterface
{
private:

    TOIDecMDPDiscrete *_m_problem;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserTOIDecMDPDiscrete(TOIDecMDPDiscrete
                            *problem);

    void Parse();

};


#endif /* !_PARSERTOIDECMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
