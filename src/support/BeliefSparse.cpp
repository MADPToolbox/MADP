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

#include "BeliefSparse.h"
#include "BeliefIteratorSparse.h"
#include "BeliefIteratorGeneric.h"
#include <float.h>
#include "StateDistributionVector.h"

#define BeliefSparse_CheckAndAbort 0

using namespace std;

BeliefSparse::BeliefSparse()
{
}

BeliefSparse::BeliefSparse(size_t size)
{
    _m_b.resize(size,false);
}

BeliefSparse::BeliefSparse(const vector<double> &belief)
{
    _m_b.resize(belief.size(),false);

    unsigned int i=0;
    for(vector<double>::const_iterator it=belief.begin();
        it!=belief.end(); ++it)
    {
        if(*it>0)
            _m_b[i]=*it;
        ++i;
    }
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}


BeliefSparse::BeliefSparse(const BeliefInterface &belief)
{
    Set(belief);
}
BeliefSparse::BeliefSparse(const StateDistribution& belief)
{
    this->Set(belief);
}


//Destructor
BeliefSparse::~BeliefSparse()
{
}

BeliefSparse& 
BeliefSparse::operator= (const BeliefSparse& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    _m_b = o._m_b;
    return *this;
}

BeliefInterface& 
BeliefSparse::operator= (const BeliefInterface& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    const BeliefSparse& casted_o = 
        dynamic_cast<const BeliefSparse&>(o);
    return(operator=(casted_o));// call the operator= for BeliefSparse
}

void BeliefSparse::Set(const BS &belief)
{
    _m_b=belief;
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

void BeliefSparse::Set(const vector<double> &belief)
{
    _m_b.resize(belief.size(),false);
    _m_b.clear();

    unsigned int i=0;
    for(vector<double>::const_iterator it=belief.begin();
        it!=belief.end(); ++it)
    {
        if(*it>0)
            _m_b[i]=*it;
        ++i;
    }
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

void BeliefSparse::Set(const StateDistribution& belief)
{
    _m_b.resize(belief.GetNrStates(),false);
    _m_b.clear();
    
    for(unsigned int i=0;i!=belief.GetNrStates();++i)
    {
        double p=belief.GetProbability(i);
        if(p>0)
            _m_b[i]=p;
    }

}


void BeliefSparse::Set(const BeliefInterface &belief)
{
    _m_b.resize(belief.Size(),false);
    _m_b.clear();

    for(unsigned int i=0; i!=belief.Size(); ++i)
    {
        if(belief.Get(i)>0)
            _m_b[i]=belief.Get(i);
    }
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
}

void BeliefSparse::Clear()
{
    _m_b.clear();
}

string BeliefSparse::SoftPrint() const
{
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
    stringstream ss;
    ss << "[" << _m_b.size() << "]< ";
    for(BScit it=_m_b.begin();
        it!=_m_b.end(); ++it)
        ss << it.index() << ":" << *it << " ";
    ss << ">";
    return(ss.str());
}

bool BeliefSparse::SanityCheck() const
{
    // check for negative and entries>1
    double sum=0;
    for(BScit it=_m_b.begin();
        it!=_m_b.end(); ++it)
    {
        if(*it<0)
            return(false);
        if(*it>1 + PROB_PRECISION)
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

double BeliefSparse::InnerProduct(const vector<double> &values) const
{
#if BeliefSparse_CheckAndAbort
    if(!SanityCheck())
        abort();
#endif
    double x=0;
    for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
        x+=(*it)*values[it.index()];

    return(x);
}

vector<double> BeliefSparse::InnerProduct(const VectorSet &v) const
{
    vector<double> values(v.size1());

    double x;
    for(unsigned int k=0;k!=v.size1();++k)
    {
        x=0;
        for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
            x+=(*it)*v(k,it.index());
        values[k]=x;
    }

    return(values);
}

vector<double> BeliefSparse::InnerProduct(const VectorSet &v,
                                          const vector<bool> &mask) const
{
    vector<double> values(v.size1(),-DBL_MAX);

    double x;
    for(unsigned int k=0;k!=v.size1();++k)
    {
        if(mask[k])
        {
            x=0;
            for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                x+=(*it)*v(k,it.index());
            values[k]=x;
        }
    }

    return(values);
}

BeliefIteratorGeneric BeliefSparse::GetIterator() const
{
    return(BeliefIteratorGeneric(new BeliefIteratorSparse(this))); 
}
