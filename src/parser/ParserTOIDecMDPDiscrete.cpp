/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ParserTOIDecMDPDiscrete.h"
#include "ParserTOIDecPOMDPDiscrete.h"

//Default constructor
ParserTOIDecMDPDiscrete::
ParserTOIDecMDPDiscrete(TOIDecMDPDiscrete
                        *problem) :
    _m_problem(problem)
{
}

void ParserTOIDecMDPDiscrete::Parse()
{
    ParserTOIDecPOMDPDiscrete parser(_m_problem);
    parser.Parse();

    _m_problem->CreateStateObservations();
}
