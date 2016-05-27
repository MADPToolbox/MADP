/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "NamedDescribedEntity.h"
#include <sstream>

using namespace std;

NamedDescribedEntity::NamedDescribedEntity(const string &name,
                                           const string &description) :
    _m_name(name),
    _m_description(description)
{
}

string NamedDescribedEntity::SoftPrint() const 
{ 
    stringstream ss;
    ss << "name:" << this->GetName() 
       << " - descr." << this->GetDescription(); 
    return(ss.str());
}

string NamedDescribedEntity::SoftPrintBrief() const
{
    return(GetName());
}
