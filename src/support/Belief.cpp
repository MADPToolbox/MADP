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

#include "Belief.h"
#include "BeliefIterator.h"
#include "BeliefIteratorGeneric.h"
#include <float.h>

#include "StateDistributionVector.h"

using namespace std;

#define Belief_CheckAndAbort 0

Belief::Belief(size_t size)
{
    _m_b.assign(size,0);
}

Belief::Belief(const vector<double> &belief)
{
    _m_b=belief;
#if Belief_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

Belief::Belief(const BeliefInterface &belief)
{
    _m_b.resize(belief.Size());
    for(unsigned int i=0; i!=belief.Size(); ++i)
        _m_b[i]=belief.Get(i);

#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}
Belief::Belief(const StateDistribution& belief)
{
    this->Set(belief);
}


//Destructor
Belief::~Belief()
{
}

Belief& 
Belief::operator= (const Belief& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    _m_b = o._m_b;
    return *this;
}

BeliefInterface& 
Belief::operator= (const BeliefInterface& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    const Belief& casted_o = dynamic_cast<const Belief&>(o);
    return(operator=(casted_o));// call the operator= for Belief
}    

void Belief::Set(const vector<double> &belief)
{
    _m_b.resize(belief.size());
    _m_b=belief;
#if Belief_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

void Belief::Set(const StateDistribution& belief)
{
    const StateDistributionVector *b;
    if((b=dynamic_cast<const StateDistributionVector*>( &belief )))
        _m_b=*b;
    else
        Set(belief.ToVectorOfDoubles());
}

void Belief::Set(const BeliefInterface &belief)
{
    _m_b.resize(belief.Size());
    _m_b.clear();

    for(unsigned int i=0; i!=belief.Size(); ++i)
        _m_b[i]=belief.Get(i);

#if Belief_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

void Belief::Clear()
{
    _m_b.clear();
}

string Belief::SoftPrint() const
{
#if Belief_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
    return(SoftPrintVector( (vector<double>) _m_b ));
}

bool Belief::SanityCheck() const
{
    // check for negative and entries>1
    double sum=0;
    for(vector<double>::const_iterator it=_m_b.begin();
        it!=_m_b.end(); ++it)
    {
        if(*it<0)
            return(false);
        if(*it>1)
            return(false);
        if(std::isnan(*it))
            return(false);
        sum+=*it;
    }

    // check if sums to 1
    if(abs(sum-1)>PROB_PRECISION)
        return(false);

    // check whether the size is not zero
    if(_m_b.size()==0)
        return(false);

    // if we haven't returned yet, the belief is fine
    return(true);
}

double Belief::InnerProduct(const vector<double> &values) const
{
#if Belief_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif

    double x=0;
    for(unsigned int s=0; s!=_m_b.size(); ++s)
        x+=_m_b[s]*values[s];

    return(x);
}

vector<double> Belief::InnerProduct(const VectorSet &v) const
{
    vector<double> values(v.size1());

    double x;
    for(unsigned int k=0;k!=v.size1();++k)
    {
        x=0;
        for(unsigned int s=0; s!=_m_b.size(); ++s)
            x+=_m_b[s]*v(k,s);
        values[k]=x;
    }

    return(values);
}

vector<double> Belief::InnerProduct(const VectorSet &v,
                                    const vector<bool> &mask) const
{
    vector<double> values(v.size1(),-DBL_MAX);

    double x;
    for(unsigned int k=0;k!=v.size1();++k)
    {
        if(mask[k])
        {
            x=0;
            for(unsigned int s=0; s!=_m_b.size(); ++s)
                x+=_m_b[s]*v(k,s);
            values[k]=x;
        }
    }

    return(values);
}

BeliefIteratorGeneric Belief::GetIterator() const
{
    return(BeliefIteratorGeneric(new BeliefIterator(this)));
}
