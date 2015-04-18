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

#include "TypeCluster.h"

using namespace std;

//Default constructor
TypeCluster::TypeCluster(Index i)
    :
        _m_i(i)
{
}
//Copy constructor.    
TypeCluster::TypeCluster(const TypeCluster& o) :
    _m_i(o._m_i)
{
    type_ci it = o._m_types.begin();
    type_ci last = o._m_types.end();

    while(it != last)
    {
        ///check that this calles the derived class copy constructor!
        Type* t = (*it)->Clone();
        _m_types.insert(t);
        it++;
    }
}

//Destructor
TypeCluster::~TypeCluster()
{
    //free the memory for all types:
    type_ci it = _m_types.begin();
    type_ci last = _m_types.end();

    while(it != last)
    {
        delete *it;
        it++;
    }

}
//Copy assignment operator
TypeCluster& TypeCluster::operator= (const TypeCluster& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    //make a deep copy:
    type_ci it = o._m_types.begin();
    type_ci last = o._m_types.end();

    while(it != last)
    {
        ///check that this calles the derived class copy constructor!
        Type* t = (*it)->Clone();
        _m_types.insert(t);
        it++;
    }
    _m_i = o._m_i;


    return *this;
}

string TypeCluster::SoftPrint() const
{
    stringstream ss;
    ss << "tcI"<<_m_i<<"={ ";

    type_ci it = _m_types.begin();
    type_ci last = _m_types.end();

    while(it != last)
    {
        ss << (*it)->SoftPrint();        
        it++;
        if(it != last)
            ss << " | ";       
    }
    ss << " }";
    return(ss.str());
}
