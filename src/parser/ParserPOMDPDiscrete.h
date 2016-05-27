/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARSERPOMDPDISCRETE_H_
#define _PARSERPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "POMDPDiscrete.h"
#include "ParserInterface.h"


/**\brief \deprecated ParserPOMDPDiscrete is a parser for
 * POMDPDiscrete that makes use of Tony Cassandra's POMDPsolve to do the parsing.*/
class ParserPOMDPDiscrete :
    public ParserInterface
{
private:    

    POMDPDiscrete *_m_problem;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ParserPOMDPDiscrete(POMDPDiscrete *problem);

    void Parse();

};


#endif /* !_PARSERPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
