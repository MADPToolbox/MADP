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

#include "BeliefValue.h"
#include <float.h>
#include <limits.h>

using namespace std;

#include "AlphaVector.h"
#include "JointBeliefInterface.h"
#include "BeliefSetNonStationary.h"

vector<double> BeliefValue::GetValues(const BeliefSet &Beliefs,
                                      const AlphaVector &alpha)
{
    ValueFunctionPOMDPDiscrete V;
    V.push_back(alpha);
    return(GetValues(Beliefs,V));
}

/**
 * Implements equation (3.5) of PhD thesis Matthijs.
 */
vector<double> BeliefValue::GetValues(const BeliefSet &Beliefs,
                                      const ValueFunctionPOMDPDiscrete &V)
{
    int nrInV=V.size();
    int nrB=Beliefs.size();

    vector<double> values(nrB,-DBL_MAX);
    double x;

    for(int b=0;b!=nrB;b++)
        for(int i=0;i!=nrInV;i++)
        {
            // compute inner product of belief with vector
            x=Beliefs[b]->InnerProduct(V[i].GetValues());

            // keep the maximizing value
            if(x>values[b])
                values[b]=x;
        }
    
    return(values);
}

vector<double> BeliefValue::GetValues(const BeliefSet &Beliefs,
                                      const QFunctionsDiscrete &Q)
{
    vector<double> values(Beliefs.size()),
        maxValues(Beliefs.size(),-DBL_MAX);
    for(QFDcit i=Q.begin();i!=Q.end();++i)
    {
        values=GetValues(Beliefs,*i);
        for(unsigned int j=0;j!=values.size();++j)
            if(values[j]>maxValues[j])
                maxValues[j]=values[j];
    }

    return(maxValues);
}

vector<double> BeliefValue::GetValues(const BeliefSetNonStationary &Beliefs,
                                      const QFunctionsDiscreteNonStationary &Q)
{
    vector<double> values(Beliefs.Size());
    Index i=0;
    for(Index t=0;t!=Q.size();++t)
    {
        vector<double> valuesT=GetValues(Beliefs.Get(t),
                                         Q[t]);
        for(Index k=0;k!=valuesT.size();++k)
        {
            values[i]=valuesT[k];
            i++;
        }
    }
    return(values);
}

vector<double> BeliefValue::GetValues(const BeliefSetNonStationary &Beliefs,
                                      const ValueFunctionPOMDPDiscrete &V)
{
    vector<double> values(Beliefs.Size());
    Index i=0;
    for(Index t=0;t!=Beliefs.GetNumberOfTimeSteps();++t)
    {
        vector<double> valuesT=GetValues(Beliefs.Get(t),V);
        for(Index k=0;k!=valuesT.size();++k)
        {
            values[i]=valuesT[k];
            i++;
        }
    }
    return(values);
}


double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const AlphaVector &alpha)
{
    return(Belief.InnerProduct(alpha.GetValues()));
}

double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const QFunctionsDiscrete &Q)
{
    double x,maxVal=-DBL_MAX;

    for(QFDcit i=Q.begin();i!=Q.end();++i)
    {
        x=GetValue(Belief,*i);
        if(x>maxVal)
            maxVal=x;
    }

    return(maxVal);    
}

double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const QFunctionsDiscreteNonStationary &Q,
                             Index t)
{
#if 0
    double x,maxVal=-DBL_MAX;

    for(QFDNScit i=Q.begin();i!=Q.end();++i)
    {
        x=GetValue(Belief,*i);
        if(x>maxVal)
            maxVal=x;
    }

    return(maxVal);
#endif
    return(GetValue(Belief,Q.at(t)));
}

double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const ValueFunctionPOMDPDiscrete &V)
{
    double x,maxVal=-DBL_MAX;

    for(unsigned int i=0;i!=V.size();i++)
    {
        // compute inner product of belief with vector
        x=Belief.InnerProduct(V[i].GetValues());
        
        // keep the maximizing value
        if(x>maxVal)
            maxVal=x;
    }

    return(maxVal);    
}

double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const ValueFunctionPOMDPDiscrete &V,
                             const vector<bool> mask)
{
    double x,maxVal=-DBL_MAX;
    bool maskValid=false;

    if(mask.size()!=V.size())
    {
        throw(E("BeliefValue::GetValue: mask has incorrect size"));
    }

    for(unsigned int i=0;i!=V.size();i++)
    {
        if(mask[i])
        {
            maskValid=true;
            // compute inner product of belief with vector
            x=Belief.InnerProduct(V[i].GetValues());
        
            // keep the maximizing value
            if(x>maxVal)
                maxVal=x;
        }
    }

    return(maxVal);
}

double BeliefValue::GetValue(const BeliefInterface &Belief,
                             const VectorSet &v,
                             const vector<bool> mask)
{
    double x,maxVal=-DBL_MAX;
    bool maskValid=false;
    vector<double> values(v.size2());

    if(mask.size()!=v.size1())
    {
        throw(E("BeliefValue::GetValue: mask has incorrect size"));
    }

    for(unsigned int i=0;i!=v.size1();i++)
    {
        if(mask[i])
        {
            maskValid=true;
            for(unsigned int k=0;k!=v.size2();++k)
                values[k]=v(i,k);

            // compute inner product of belief with vector
            x=Belief.InnerProduct(values);

            // keep the maximizing value
            if(x>maxVal)
                maxVal=x;
        }
    }

    return(maxVal);
}

int
BeliefValue::GetMaximizingVectorIndex(const BeliefInterface &b, 
                                      const VectorSet &v)
{
    int nrInV=v.size1();
    int maximizingVectorI=0;
    double maxVal;

    // compute value of b for every vector in VS
    vector<double> Vb=b.InnerProduct(v);

    // find maximizing vector, ignores effects of duplicate values
    maxVal=-DBL_MAX;
    for(int k=0;k!=nrInV;k++)
    {
        if(Vb[k]>maxVal)
        {
            maxVal=Vb[k];
            maximizingVectorI=k;
        }
    }

    return(maximizingVectorI);
}

int
BeliefValue::
GetMaximizingVectorIndex(const BeliefInterface &b, 
                         const ValueFunctionPOMDPDiscrete &V)
{
    int nrInV=V.size();
    int maximizingVectorI=0;
    double maxVal;

    // compute value of b for every vector in V
    vector<double> Vb(nrInV,0);
    for(int k=0;k!=nrInV;k++)
        Vb[k]=b.InnerProduct(V[k].GetValues());

    // find maximizing vector, ignores effects of duplicate values
    maxVal=-DBL_MAX;
    for(int k=0;k!=nrInV;k++)
    {
        if(Vb[k]>maxVal)
        {
            maxVal=Vb[k];
            maximizingVectorI=k;
        }
    }

    return(maximizingVectorI);
}

/** If no vector has its mask enabled, the function returns -1.
 */
int
BeliefValue::GetMaximizingVectorIndex(const BeliefInterface &b, 
                                      const VectorSet &v,
                                      const vector<bool> &mask)
{
    int nrInV=v.size1();
    int maximizingVectorI=INT_MAX;
    double maxVal;
    bool maskValid=false;

    if(static_cast<int>(mask.size())!=nrInV)
    {
        throw(E("BeliefValue::GetMaximizingVectorIndex: mask has incorrect size"));
    }

    for(int k=0;k!=nrInV;k++)
        if(mask[k])
        {
            maskValid=true;
            break;
        }

    // no vector was enabled in the mask, so there is no maximizing vector
    if(!maskValid)
        return(-1);

    // compute value of b for every vector in VS which has its mask
    // set to true
    vector<double> Vb=b.InnerProduct(v,mask);

    // find maximizing vector, ignores effects of duplicate values
    maxVal=-DBL_MAX;
    for(int k=0;k!=nrInV;k++)
    {
        if(mask[k] && Vb[k]>maxVal)
        {
            maxVal=Vb[k];
            maximizingVectorI=k;
        }
    }

    if(maxVal==-DBL_MAX)
    {
         throw(E("BeliefValue::GetMaximizingVectorIndex: no maximizing vector found"));
    }
    return(maximizingVectorI);
}

int 
BeliefValue::GetMaximizingVectorIndexAndValue(const BeliefInterface &b, 
                                              const VectorSet &v,
                                              const vector<bool> &mask,
                                              double &value)
{
    //ugly, copy of JointBelief version..

    int nrInV=v.size1();
    int maximizingVectorI=INT_MAX;
    bool maskValid=false;

    if(static_cast<int>(mask.size())!=nrInV)
    {
        throw(E("BeliefValue::GetMaximizingVectorIndex: mask has incorrect size"));
    }

    for(int k=0;k!=nrInV;k++)
        if(mask[k])
        {
            maskValid=true;
            break;
        }

    // no vector was enabled in the mask, so there is no maximizing vector
    if(!maskValid)
        return(-1);

    // compute value of b for every vector in VS which has its mask
    // set to true
    vector<double> Vb=b.InnerProduct(v,mask);

    // find maximizing vector, ignores effects of duplicate values
    value=-DBL_MAX;
    for(int k=0;k!=nrInV;k++)
    {
        if(mask[k] && Vb[k]>value)
        {
            value=Vb[k];
            maximizingVectorI=k;
        }
    }

    if(value==-DBL_MAX)
    {
         throw(E("BeliefValue::GetMaximizingVectorIndex: no maximizing vector found"));
    }
    return(maximizingVectorI);
}

AlphaVector
BeliefValue::GetMaximizingVector(const BeliefSet &S,
                                 int k,
                                 const ValueFunctionPOMDPDiscrete &V)
{
    int nrInV=V.size();

    double x,y=-DBL_MAX;
    int maxI=-1;

    for(int i=0;i!=nrInV;i++)
    {
        x=S[k]->InnerProduct(V[i].GetValues());

        if(x>y)
        {
            y=x;
            maxI=i;
        }
    }

    if(maxI!=-1)
        return(V[maxI]);
    else
    {
        AlphaVector alpha(S[k]->Size(),-DBL_MAX);
        alpha.SetAction(INT_MAX);
        return(alpha);
    }
}

