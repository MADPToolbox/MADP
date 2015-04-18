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

#include "QFunctionForDecPOMDP.h"
#include "directories.h"
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

string QFunctionForDecPOMDP::GetCacheFilename() const
{
    stringstream ss;
    ss << directories::MADPGetResultsDir("GMAA",GetPU())
       << "/" << SoftPrintBrief() << "heuristic_h" << GetPU()->GetHorizon();
    if(GetPU()->GetDiscount()!=1)
        ss << "_g" << GetPU()->GetDiscount();
    return(ss.str());
}
