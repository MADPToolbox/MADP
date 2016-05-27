/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MADPParser.h"
#include "ParserDPOMDPFormat_Spirit.h"
#include "ParserTOIDecPOMDPDiscrete.h"
#include "ParserTOIDecMDPDiscrete.h"
#include "ParserTOIFactoredRewardDecPOMDPDiscrete.h"
#include "ParserTOICompactRewardDecPOMDPDiscrete.h"
#include "ParserPOMDPFormat_Spirit.h"
#include "ParserProbModelXML.h"
#include "DecPOMDPDiscrete.h"
#include "TOIDecPOMDPDiscrete.h"
#include "TOIDecMDPDiscrete.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "TOICompactRewardDecPOMDPDiscrete.h"
#include "POMDPDiscrete.h"

void MADPParser::Parse(DecPOMDPDiscrete *model)
{
    DPOMDPFormatParsing::ParserDPOMDPFormat_Spirit parser(model);
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
    POMDPFormatParsing::ParserPOMDPFormat_Spirit parser(model);
    parser.Parse();
}
