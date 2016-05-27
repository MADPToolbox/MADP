/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "StringTools.h"

using namespace std;

namespace StringTools{
    
string Append(const std::string& ioString, int inValue)
{
    std::ostringstream o;
    o << ioString << inValue;
    return o.str();
}

string Trim(const std::string& ioString)
{
    string trimmed = ioString;
    size_t pos = trimmed.find_last_not_of(" \t");
    if(pos < trimmed.length()-1 && pos > 0)
      trimmed.erase(pos+1);
    return trimmed;
}

}

