/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "Type_AOHIndex.h"

using namespace std;

/*
//Default constructor
Type_AOHIndex::Type_AOHIndex()
{
}
//Copy constructor.    
Type_AOHIndex::Type_AOHIndex(const Type_AOHIndex& o) 
{
}
//Destructor
Type_AOHIndex::~Type_AOHIndex()
{
}
*/
//Copy assignment operator
Type_AOHIndex& Type_AOHIndex::operator= (const Type_AOHIndex& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    _m_aohI = o._m_aohI;
    //call base class
    Type::operator=(o);

    return *this;
}


Type* Type_AOHIndex::Clone() const
{
    Type_AOHIndex* t = new Type_AOHIndex(*this);
    return (Type*) t;
}

string Type_AOHIndex::SoftPrint() const
{
    stringstream ss;
    ss << "aoh"<<_m_aohI;
    return(ss.str());
}
