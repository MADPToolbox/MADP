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

#include "AlphaVector.h"
#include <iostream>

using namespace std;

/// Default constructor
AlphaVector::AlphaVector()
{
    _m_values.assign(0,0);
    _m_betaI=-1;
}

AlphaVector::AlphaVector(size_t nrS)
{
    _m_values.assign(nrS,0);
    _m_betaI=-1;
}

AlphaVector::AlphaVector(size_t nrS, double val)
{
    _m_values.assign(nrS,val);
    _m_betaI=-1;
}

//Destructor
AlphaVector::~AlphaVector()
{
}
//Copy assignment operator
AlphaVector& AlphaVector::operator= (const AlphaVector& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    this->_m_action=o._m_action;
    this->_m_values=o._m_values;
    this->_m_betaI=o._m_betaI;

    return *this;
}

std::ostream & 
operator<< (std::ostream& o, const AlphaVector& v)
{
    o << v.SoftPrint();
    return o;
}

string AlphaVector::SoftPrint() const
{
    stringstream ss;
    ss << "a " << _m_action << " bI " << _m_betaI
       << " : values";
    vector<double>::const_iterator it=_m_values.begin();
    while(it!=_m_values.end())
    {
        ss << " " << *it;
        it++;
    }
    return(ss.str());
}

void AlphaVector::SetValues(const vector<double> &vs)
{
    if(vs.size()!=_m_values.size())
        throw(E("AlphaVector::SetValues vector sizes do not match"));
    else
        _m_values=vs;
}
    
bool AlphaVector::Equal(const AlphaVector &alpha) const
{
    if(this->GetAction()!=alpha.GetAction())
        return(false);

    if(this->GetBetaI()!=alpha.GetBetaI())
        return(false);

    if(!EqualValues(alpha))
        return(false);

    // else
    return(true);
}

bool AlphaVector::EqualValues(const AlphaVector &alpha) const
{
    if(this->GetNrValues()!=alpha.GetNrValues())
        return(false);

    const vector<double> &values=alpha.GetValues();

    if(_m_values!=values)
        return(false);

    // else
    return(true);
}

/// Add B to this and return as a new vector
AlphaVector AlphaVector::Add(const AlphaVector& B) const
{
    //make a copy of this
    AlphaVector out(*this);
    out.IAdd(B);
#if 0
    cout << "AlphaVector:Add - adding this=<" << *this << "> and B=<" << B << ">, out= <" <<out<< ">" << endl;
#endif
    return out;
}
/// Add B to this and return as a new vector
void AlphaVector::IAdd(const AlphaVector& B)
{
    if(this->GetNrValues() != B.GetNrValues())
        throw(E("adding AlphaVectors of different size?!"));
    for(Index sI=0; sI < this->GetNrValues(); sI++)
        _m_values[sI] += B._m_values[sI];
}
