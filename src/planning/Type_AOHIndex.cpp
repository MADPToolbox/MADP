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
