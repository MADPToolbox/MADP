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
