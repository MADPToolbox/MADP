/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MADPParser.h"
#include "ParserDecPOMDPDiscrete.h"
#include "ParserTOIDecPOMDPDiscrete.h"
#include "ParserTOIDecMDPDiscrete.h"
#include "ParserTOIFactoredRewardDecPOMDPDiscrete.h"
#include "ParserTOICompactRewardDecPOMDPDiscrete.h"
#include "ParserPOMDPDiscrete.h"
#include "ParserProbModelXML.h"
#include "DecPOMDPDiscrete.h"
#include "TOIDecPOMDPDiscrete.h"
#include "TOIDecMDPDiscrete.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "TOICompactRewardDecPOMDPDiscrete.h"
#include "POMDPDiscrete.h"

void MADPParser::Parse(DecPOMDPDiscrete *model)
{
    ParserDecPOMDPDiscrete parser(model);
    parser.Parse();
}

void MADPParser::Parse(TOIDecPOMDPDiscrete *model)
{
    ParserTOIDecPOMDPDiscrete parser(model);
    parser.Parse();
}

void MADPParser::Parse(TOIDecMDPDiscrete *model)
{
    ParserTOIDecMDPDiscrete parser(model);
    parser.Parse();
}

void MADPParser::Parse(TOIFactoredRewardDecPOMDPDiscrete *model)
{
    ParserTOIFactoredRewardDecPOMDPDiscrete parser(model);
    parser.Parse();
}

void MADPParser::Parse(TOICompactRewardDecPOMDPDiscrete *model)
{
    ParserTOICompactRewardDecPOMDPDiscrete parser(model);
    parser.Parse();
}

void MADPParser::Parse(FactoredDecPOMDPDiscrete *model)
{
    ParserProbModelXML parser(model);
    parser.Parse();
}

void MADPParser::Parse(POMDPDiscrete *model)
{
    ParserPOMDPDiscrete parser(model);
    parser.Parse();
}
