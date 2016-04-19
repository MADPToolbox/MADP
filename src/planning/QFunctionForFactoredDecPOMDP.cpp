/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
