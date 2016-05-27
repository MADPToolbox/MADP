/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
