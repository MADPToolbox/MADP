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
