/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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

#include "QFunctionForFactoredDecPOMDP.h"
#include "directories.h"

using namespace std;

////Default constructor
//QFunctionForFactoredDecPOMDP::QFunctionForFactoredDecPOMDP()
//{
//}
////Copy constructor.    
//QFunctionForFactoredDecPOMDP::QFunctionForFactoredDecPOMDP(const QFunctionForFactoredDecPOMDP& o) 
//{
//}
////Destructor
//QFunctionForFactoredDecPOMDP::~QFunctionForFactoredDecPOMDP()
//{
//}
////Copy assignment operator
//QFunctionForFactoredDecPOMDP& QFunctionForFactoredDecPOMDP::operator= (const QFunctionForFactoredDecPOMDP& o)
//{
    //if (this == &o) return *this;   // Gracefully handle self assignment
    //// Put the normal assignment duties here...

    //return *this;
//}

string QFunctionForFactoredDecPOMDP::GetCacheFilename() const
{
    // would be nice if this included the BackupScopes parameter as well...
    stringstream ss;
    ss << directories::MADPGetResultsDir("GMAA",GetPU())
       << "/" << SoftPrintBrief() << "heuristic_h" << GetPU()->GetHorizon();
    if(GetPU()->GetDiscount()!=1)
        ss << "_g" << GetPU()->GetDiscount();
    return(ss.str());
}
