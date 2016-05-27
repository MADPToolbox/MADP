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
    _m_planningUnitDecPOMDPDiscrete(&p), 
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
    _m_planningUnitDecPOMDPDiscrete(p),
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


void ValueFunctionDecPOMDPDiscrete::SetCached(Index sI, Index JOHI)
{
    pair<Index, bool> p(GetCombinedIndex(sI, JOHI), true);
    pair<map<Index,bool>::iterator, bool> result = _m_cached.insert( p );
    if(result.second != true)
    {
        // can't insert if this key ( GetCombinedIndex(sI, JOHI) ) is already
        // present - in this case, we need to change the value to true...
        // *(result.first) isa pair<int,bool>, so we do:
        (*(result.first)).second = true;
    }
}

double ValueFunctionDecPOMDPDiscrete::CalculateV(bool cache) 
{
    if(cache)
        return CalculateV0RecursivelyCached();
    else
        return CalculateV0RecursivelyNotCached();
}

double ValueFunctionDecPOMDPDiscrete::CalculateV0RecursivelyCached()
{
    CreateV();
#if DEBUG_CALCV
    if(DEBUG_CALCV_CACHE)
        cout << "CalculateV0Recursively::creating a new value function"
             << endl;
    if(DEBUG_CALCV) {cout << "evaluating joint policy:\n"; GetJPol()->Print();}
#endif                 

    double val = 0;
    for(Index sI = 0; sI < _m_nrS; sI++)
    {
        double v_sI = CalculateVsjohRecursivelyCached(sI, 
                Globals::INITIAL_JOHI, 0 );
#if DEBUG_CALCV
        if(DEBUG_CALCV){
            cout << ">>>ValueFunctionDecPOMDPDiscrete::CalculateV() -"
                 << " CalculateVsjohRecursively(sI=" << sI
                 << ", INITIAL_JOHI, cache="<< cache<< ") = " << v_sI << endl; }
#endif                 
        val +=  _m_pu->GetInitialStateProbability(sI) * v_sI;
    }
#if DEBUG_CALCV
    if(DEBUG_CALCV) {  cout << "This policy's V=" << val <<endl;}
#endif                 
    return val;
}



double ValueFunctionDecPOMDPDiscrete::CalculateV0RecursivelyNotCached()
{
#if DEBUG_CALCV
    if(DEBUG_CALCV) {cout << "evaluating joint policy:\n"; GetJPol()->Print();}
#endif                 

    double val = 0;
    for(Index sI = 0; sI < _m_nrS; sI++)
    {
        double v_sI = CalculateVsjohRecursivelyNotCached(sI, 
                Globals::INITIAL_JOHI, 0);
#if DEBUG_CALCV
        if(DEBUG_CALCV){
            cout << ">>>ValueFunctionDecPOMDPDiscrete::CalculateV() -"
                 << " CalculateVsjohRecursively(sI=" << sI
                 << ", INITIAL_JOHI, cache="<< cache<< ") = " << v_sI << endl; }
#endif                 
        val +=  _m_pu->GetInitialStateProbability(sI) * v_sI;
    }
#if DEBUG_CALCV
    if(DEBUG_CALCV) {  cout << "This policy's V=" << val <<endl;}
#endif                 
    return val;
}




double ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursivelyCached(Index sI, 
    Index johI, Index stage)
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
            if(IsCached(sucSI, sucJohI))
                thisSucV =  (*_m_p_V)(sucSI, sucJohI);
            else
                thisSucV=CalculateVsjohRecursivelyCached(sucSI,sucJohI,stage+1);
            ExpFutureR += probSucSI * probJOI * thisSucV;
        }//end for each observation
    }//end for each potential succesor state
    double val = R + ExpFutureR;
    (*_m_p_V)(sI, johI) = val;
    SetCached(sI, johI);
#if DEBUG_CALCV
    cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
         << sI <<", " << johI << ") \n->immediate R="<<R<<
        " \n->exp. future reward="<<ExpFutureR<<"\n->V="<<val<<endl;
#endif                 
    return(val);
}


double ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursivelyNotCached(
        Index sI, Index johI, Index stage)
{
#if DEBUG_CALCV
    {cout<< "\nValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
         << sI << ", " << johI << ") called"<<endl;}
#endif         

    Index jaI = GetJPol()->GetJointActionIndex(johI);
    double R = GetPU()->GetReward(sI, jaI);
    double ExpFutureR = 0.0;
    // horizon h policy makes decision on observation histories which 
    // maximally have length h-1. 
    // t=0 - () - length=0 
    // ... 
    // t=h-1 - (o1,...,o{h-1}) - length h-1
    if(_m_pu->GetTimeStepForJOHI(johI) >= _m_h - 1 )
    {
#if DEBUG_CALCV
        {        cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjoh"
                 << "Recursively("<< sI <<", " << johI << ") - V="<<R<<endl;} 
#endif                 
        return(R);
    }

#if DEBUG_CALCV
    if(DEBUG_CALCV){ cout << "Calculating future reward"<<endl;}
#endif                 
    for(Index sucSI = 0; sucSI < _m_nrS; sucSI++)
    {
        double probSucSI = _m_pu->GetTransitionProbability(sI, jaI,
               sucSI);
#if DEBUG_CALCV
        if(DEBUG_CALCV){ cout << "P(s"<<sucSI<<"|s"<<sI<<",ja"<<jaI<<")="<<
                probSucSI<<endl;}
#endif                 

        for(Index joI = 0; joI < _m_nrJO; joI++)
        {
            double probJOI =  _m_pu->GetObservationProbability(jaI, sucSI, joI);
#if DEBUG_CALCV
            if(DEBUG_CALCV){ cout << "P(jo"<<joI<<"|ja"<<jaI<<",s"<<sucSI<<")="
                                  <<probJOI<<endl;}
#endif                 
            Index sucJohI = _m_pu->GetSuccessorJOHI(johI, joI);
            double thisSucV = CalculateVsjohRecursivelyNotCached(sucSI, 
                    sucJohI, stage+1);
            ExpFutureR += probSucSI * probJOI * thisSucV;
        }//end for each observation
    }//end for each potential succesor state
    double val = R + ExpFutureR;
#if DEBUG_CALCV
    if(DEBUG_CALCV){ 
        cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
             << sI <<", " << johI << ") \n->immediate R="<<R<<
            " \n->exp. future reward="<<ExpFutureR<<"\n->V="<<val<<endl;} 
#endif                 
    return(val);
}
