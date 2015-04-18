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

#include "Type_PointerTuple.h"
#include "TypeCluster.h"

using namespace std;

//Default constructor
Type_PointerTuple::Type_PointerTuple(const TypeCluster* pred, Index aI, Index oI)
    :
    Type(POINTERTUPLE)
    ,_m_pred(pred)
    ,_m_aI(aI)
    ,_m_oI(oI)
{
}
//Copy constructor.    
Type_PointerTuple::Type_PointerTuple(const Type_PointerTuple& o) 
    :
    Type(o)
    ,_m_pred(o._m_pred)
    ,_m_aI(o._m_aI)
    ,_m_oI(o._m_oI)
{
}
//Destructor
Type_PointerTuple::~Type_PointerTuple()
{
}
//Copy assignment operator
Type_PointerTuple& Type_PointerTuple::operator= (const Type_PointerTuple& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    Type::operator=(o);
    _m_pred = o._m_pred;
    _m_aI = o._m_aI;
    _m_oI = o._m_oI;
    return *this;
}

Type* Type_PointerTuple::Clone() const
{
    Type_PointerTuple* t = new Type_PointerTuple(*this);
    return (Type*) t;
}

string Type_PointerTuple::SoftPrint() const
{
    stringstream ss;
    ss <<"(";
    if(_m_pred != 0)
    {
        ss << _m_pred->SoftPrint();
    }
    ss << "aI=" << _m_aI << ",oI="<< _m_oI << ")";
    return(ss.str());
}


