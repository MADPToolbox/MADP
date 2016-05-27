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

#include "AlphaVectorPruning.h"
#include <float.h>
#include <config.h>

#ifdef HAVE_LIBLPSOLVE55_PIC
extern "C" {
#include <lpsolve/lp_lib.h>
}
#endif

using namespace std;

#define DEBUG_AlphaVectorPruning 0

AlphaVectorPruning::AlphaVectorPruning()
{
}

AlphaVectorPruning::~AlphaVectorPruning()
{
}

ValueFunctionPOMDPDiscrete AlphaVectorPruning::Prune(const ValueFunctionPOMDPDiscrete &Vin)
{
    if(Vin.size()==0)
        return(Vin);

    // remove alpha vectors that are dominated by a single other alpha
    // vector, can be done by checking only the corners of the belief
    // simplex
    ValueFunctionPOMDPDiscrete in = ParetoPrune(Vin);

#if DEBUG_AlphaVectorPruning
#if 0
    cout << "AlphaVectorPruning original value function: ";
    for(VFPDcit j=Vin.begin();j!=Vin.end();++j)
        cout << SoftPrintVector(j->GetValues()) << " ";
    cout << endl;
#endif
    cout << "AlphaVectorPruning after ParetoPrune (reduced from " << Vin.size()
         << " to " << in.size() << " vectors):" << endl;
    for(VFPDcit j=in.begin();j!=in.end();++j)
        cout << SoftPrintVector(j->GetValues()) << endl;
#endif

    ValueFunctionPOMDPDiscrete all(in);
    ValueFunctionPOMDPDiscrete Vpruned;

    size_t nrStates = all.at(0).GetNrValues();
		
    while(all.size()>0)
    {
        AlphaVector curr=all.at(0);
        bool foundBelief=false;
        vector<double> belief(nrStates,0);
        foundBelief = FindBelief(curr, Vpruned, belief);
        if(foundBelief)
        {
            // we found a belief for which curr is not dominated, now
            // find the corresponding vector
            double highest=-DBL_MAX;
            for (Index i=0;i!=all.size();++i)
            {
                double value=InnerProduct(all.at(i), belief);
                if( value>highest ||
                    (Globals::EqualReward(value,highest) && LexGreater(all.at(i),curr)))
                {
                    curr = all.at(i);
                    highest = value;
                }
            }
            Vpruned.push_back(curr);
#if DEBUG_AlphaVectorPruning
            cout << "AlphaVectorPruning adding vector for belief " << SoftPrintVector(belief) << ":"
                 << SoftPrintVector(curr.GetValues())
                 << endl;
#endif
            RemoveFirstOccurrence(all,curr);
        }
        else
            RemoveFirst(all);
    }
    return(Vpruned);
}


ValueFunctionPOMDPDiscrete AlphaVectorPruning::ParetoPrune(const ValueFunctionPOMDPDiscrete &Vin)
{
    ValueFunctionPOMDPDiscrete uset = Vin;
    ValueFunctionPOMDPDiscrete result;
    while(uset.size()>0)
    {
        AlphaVector u = uset.at(0);
        for(Index i=1; i<uset.size();++i)
        {
            AlphaVector v = uset.at(i);
            if(ParetoDominates(v, u))
                u=v;
        }
        ValueFunctionPOMDPDiscrete uset2=ValueFunctionPOMDPDiscrete();
        for(Index i=0; i<uset.size();++i)
        {
            AlphaVector v = uset.at(i);
            if(!ParetoDominates(u, v))
                uset2.push_back(v);
        }
        result.push_back(u);
        uset=uset2;
    }

    return(result);
}

bool AlphaVectorPruning::FindBelief(const AlphaVector &p,
                                    const ValueFunctionPOMDPDiscrete &uU,
                                    vector<double> &belief)
{
    bool foundBelief=false;
#ifdef HAVE_LIBLPSOLVE55_PIC
    size_t nrStates = p.GetNrValues();
    belief=vector<double>(nrStates,0);

    // in case the input value function is empty, we return a belief with first
    // element equal to 1.0 (in fact any belief can be returned).
    if(uU.size()==0)
    {
        belief=vector<double>(nrStates, 0.0);
        belief[0] = 1.0;
        return(true);
    }

    lprec *lp = make_lp(0, nrStates+1);
        
    set_verbose(lp, 1);
    set_maxim(lp);
    
    for(Index i=0; i<uU.size();i++)
    {
        double constraint[nrStates+2];
        
        AlphaVector curr = uU.at(i);
        for(Index j=0; j<nrStates; j++)
            constraint[j+1] = (p.GetValue(j)-curr.GetValue(j));
        
        constraint[nrStates+1]=-1;
        add_constraint(lp, constraint, ROWTYPE_GE, 0);
    }
    
    double wConstraint[nrStates+2];
    for(Index j=0; j<=nrStates; j++)
        wConstraint[j+1] = 1;
    
    wConstraint[nrStates+1]=0;
    set_unbounded(lp,nrStates+1);
    add_constraint(lp, wConstraint, ROWTYPE_EQ, 1);


    double objective[nrStates+2];
    for(Index j=0; j<=nrStates; j++)
        objective[j+1] = 0;
        
    objective[nrStates+1]=1;
    set_obj_fn(lp,objective);

#if 0
    print_lp(lp);
#endif

    solve(lp);
			
#if 0
    // print solution
    cout << "Value of objective function:" << get_objective(lp)
         << " status " << get_statustext(lp,get_status(lp)) << endl;
#endif

    if(get_status(lp)==OPTIMAL &&
       get_objective(lp) > 0.0)
    {
#if 0
        print_solution(lp,1);
#endif
        double *var;
        get_ptr_variables(lp,&var);

        foundBelief=true;

        for (Index i = 0; i < nrStates/*var.length-1*/; i++)
        {
            belief.at(i)=var[i];
        }
#if DEBUG_AlphaVectorPruning
        cout << SoftPrintVector(p.GetValues()) << " objective value "
             << get_objective(lp) << " belief "
             << SoftPrintVector(belief) << endl;
#endif
    }
    else
    {
#if 0
        cout << SoftPrintVector(p.GetValues()) << " no solution" << endl;
#endif
    }    
    // delete the problem and free memory
    delete_lp(lp);
#else // HAVE_LIBLPSOLVE55_PIC
    throw(E("AlphaVectorPruning: lpsolve was not installed"));
#endif // HAVE_LIBLPSOLVE55_PIC    
    return(foundBelief);
}

bool AlphaVectorPruning::ParetoDominates(AlphaVector x, AlphaVector y)
{
    if(x.GetNrValues()!=y.GetNrValues())
        throw(E("unequal length Pareto test"));

    for(Index i=0; i<x.GetNrValues(); ++i)
        if(y.GetValue(i)>x.GetValue(i))
            return(false);

    return(true);
}

void AlphaVectorPruning::RemoveFirst(ValueFunctionPOMDPDiscrete &V)
{
    V.erase(V.begin());
}

void AlphaVectorPruning::RemoveFirstOccurrence(ValueFunctionPOMDPDiscrete &V,
                                               const AlphaVector &alpha)
{
    ValueFunctionPOMDPDiscrete::iterator it;
    for(it=V.begin();it!=V.end();++it)
        if(alpha.Equal(*it))
        {
            V.erase(it);
            return;
        }
}

bool AlphaVectorPruning::Contains(const ValueFunctionPOMDPDiscrete &V,
                                  const AlphaVector &alpha)
{
    for(Index i=0;i!=V.size();++i)
        if(alpha.Equal(V.at(i)))
            return(true);

    return(false);
}

double AlphaVectorPruning::InnerProduct(const AlphaVector &alpha,
                                        const vector<double> &belief)
{
    double innerProduct=0;
    for(Index i=0;i!=alpha.GetNrValues();++i)
        innerProduct+=alpha.GetValue(i) * belief.at(i);

    return(innerProduct);
}

bool AlphaVectorPruning::LexGreater(const AlphaVector &alpha1,
                                    const AlphaVector &alpha2)
{
    for(Index i=0;i!=alpha1.GetNrValues();++i)
    {
        if(Globals::EqualReward(alpha1.GetValue(i),
                                alpha2.GetValue(i)))
            continue;
        else if(alpha1.GetValue(i)>alpha2.GetValue(i))
            return(true);
        else
            return(false);
    }
    return(false);
}
