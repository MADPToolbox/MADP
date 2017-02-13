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

#include "ValueFunctionDecPOMDPDiscrete.h"

#define DEBUG_CALCV 0

using namespace std;

ValueFunctionDecPOMDPDiscrete::ValueFunctionDecPOMDPDiscrete(
    PlanningUnitDecPOMDPDiscrete& p , JointPolicyDiscretePure& jp):
    _m_planningUnitDecPOMDPDiscrete(&p), //BAS is never used, _m_pu used instead
    _m_jointPolicyDiscretePure(&jp)
{
    _m_V_initialized = false;
    _m_p_V = 0;
    _m_pu = &p;
    _m_jpol = &jp;
    _m_nrJOH = _m_pu->GetNrJointObservationHistories();
    _m_nrJO = _m_pu->GetNrJointObservations();
    _m_nrS =  _m_pu->GetNrStates();
    _m_h = _m_pu->GetHorizon();
}

ValueFunctionDecPOMDPDiscrete::ValueFunctionDecPOMDPDiscrete(
    PlanningUnitDecPOMDPDiscrete* p , JointPolicyDiscretePure* jp):
    _m_planningUnitDecPOMDPDiscrete(p), //BAS is never used, _m_pu used instead
    _m_jointPolicyDiscretePure(jp)
{
    _m_V_initialized = false;
    _m_p_V = 0;
    _m_pu = p;
    _m_jpol = jp;
    _m_nrJOH = _m_pu->GetNrJointObservationHistories();
    _m_nrJO = _m_pu->GetNrJointObservations();
    _m_nrS =  _m_pu->GetNrStates();
    _m_h = _m_pu->GetHorizon();

}

//Copy assignment constructor.    
ValueFunctionDecPOMDPDiscrete::ValueFunctionDecPOMDPDiscrete(
    const ValueFunctionDecPOMDPDiscrete& o) 
{
    _m_V_initialized = false;
    _m_p_V = new Matrix(*o._m_p_V);//this makes a deep copy.
    _m_pu = o._m_pu;
    _m_jpol = o._m_jpol;
    _m_nrJOH = o._m_nrJOH;
}

//Destructor
ValueFunctionDecPOMDPDiscrete::~ValueFunctionDecPOMDPDiscrete()
{
    DeleteV();
}

void ValueFunctionDecPOMDPDiscrete::DeleteV()
{
    if(_m_V_initialized)
    {
        delete(_m_p_V);
        _m_V_initialized = false;
    }
    _m_cached.clear();
}

void ValueFunctionDecPOMDPDiscrete::CreateV()
{
    if(_m_V_initialized)
        DeleteV();

    _m_p_V = new Matrix(_m_nrS, _m_nrJOH);
    //GetPU()->GetNrStates(), _GetPU()->GetNrJointObservationHistories());
    _m_p_V->clear();
    _m_V_initialized = true;
}

inline bool ValueFunctionDecPOMDPDiscrete::IsCached(Index sI, Index JOHI)
{    
    std::set<Index>::iterator it=_m_cached.find(GetCombinedIndex(sI, JOHI));
    return it!=_m_cached.end();
}

inline void ValueFunctionDecPOMDPDiscrete::SetCached(Index sI, Index JOHI)
{
    _m_cached.insert(GetCombinedIndex(sI, JOHI));
}

template <bool CACHED>
double ValueFunctionDecPOMDPDiscrete::CalculateV()
{
   CreateV();
#if DEBUG_CALCV
   cout << "CalculateV0Recursively::creating a new value function (cache:"<<CACHED<<")"<< endl;
   cout << "evaluating joint policy:\n"; GetJPol()->Print();
#endif                 

    double val = 0;
    for(Index sI = 0; sI < _m_nrS; sI++)
    {
        
        double v_sI;
        switch (CACHED)// compile time optimized as result of CACHED being a template variable
        {
        case true:  v_sI=CalculateVsjohRecursively_cached(sI, Globals::INITIAL_JOHI, 0 );break;
        case false: v_sI=CalculateVsjohRecursively<false>(sI, Globals::INITIAL_JOHI, 0 );break;
        }

#if DEBUG_CALCV
        cout << ">>>ValueFunctionDecPOMDPDiscrete::CalculateV() -"
             << " CalculateVsjohRecursively(sI=" << sI
             << ", INITIAL_JOHI, cache="<< CACHED << ") = " << v_sI << endl; 
#endif                 
        val +=  _m_pu->GetInitialStateProbability(sI) * v_sI;
    }
#if DEBUG_CALCV
    cout << "This policy's V=" << val <<endl;
#endif                 
    return val;
}

inline double ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively_cached(Index sI,Index johI,unsigned int stage)
{
    if( IsCached(sI, johI) )
    {
#if DEBUG_CALCV
        cout << "returning cached result"<<endl;
#endif                 
        return ((*_m_p_V)(sI, johI));
    }
    else
    {
        double v=CalculateVsjohRecursively<true>(sI,johI,stage);
        SetCached(sI, johI);
        return v;
    }
}

template <bool CACHED>
double ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively(Index sI,Index johI,unsigned int stage)
{
#if DEBUG_CALCV
    cout<< "\nValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
         << sI << ", " << johI << ") called"<<endl;
#endif

    Index jaI = GetJPol()->GetJointActionIndex(johI);
    double R = _m_pu->GetReward(sI, jaI);
    double ExpFutureR = 0.0;
    // horizon h policy makes decision on observation histories which 
    // maximally have length h-1. 
    // t=0 - () - length=0 
    // ... 
    // t=h-1 - (o1,...,o{h-1}) - length h-1
    if(stage >= _m_h - 1 )
    {
#if DEBUG_CALCV
        {        cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjoh"
                 << "Recursively("<< sI <<", " << johI << ") - V="<<R<<endl;} 
#endif                 
        return(R);
    }

#if DEBUG_CALCV
    cout << "Calculating future reward"<<endl;
#endif                 
    for(Index sucSI = 0; sucSI < _m_nrS; sucSI++)
    {
        double probSucSI = _m_pu->GetTransitionProbability(sI, jaI,sucSI);
#if DEBUG_CALCV
        cout << "P(s"<<sucSI<<"|s"<<sI<<",ja"<<jaI<<")="<<probSucSI<<endl;
#endif                 

        for(Index joI = 0; joI < _m_nrJO; joI++)
        {
            double probJOI =  _m_pu->GetObservationProbability(jaI, sucSI, joI);
#if DEBUG_CALCV
            cout << "P(jo"<<joI<<"|ja"<<jaI<<",s"<<sucSI<<")="<<probJOI<<endl;
#endif                 
            Index sucJohI = _m_pu->GetSuccessorJOHI(johI, joI);
            double thisSucV;
            switch (CACHED)// compile time optimized as result of CACHED being a template variable
            {
            case true:  thisSucV=CalculateVsjohRecursively_cached(sucSI,sucJohI,stage+1);break;
            case false: thisSucV=CalculateVsjohRecursively<false>(sucSI,sucJohI,stage+1);break;
            }
            ExpFutureR += probSucSI * probJOI * thisSucV;
        }//end for each observation
    }//end for each potential succesor state
    double val = R + ExpFutureR;
    (*_m_p_V)(sI, johI) = val;
#if DEBUG_CALCV
    cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
         << sI <<", " << johI << ") \n->immediate R="<<R<<
        " \n->exp. future reward="<<ExpFutureR<<"\n->V="<<val<<endl;
#endif                 
    return(val);
}

// instantiate both template options so we don't have to move the method definition to header file
template double ValueFunctionDecPOMDPDiscrete::CalculateV<true>();
template double ValueFunctionDecPOMDPDiscrete::CalculateV<false>();

