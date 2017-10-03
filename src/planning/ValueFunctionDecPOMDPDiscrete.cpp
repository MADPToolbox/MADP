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

#define DEBUG_CALCV 0
#define DEBUG_CALCV_CACHE 0

#include "ValueFunctionDecPOMDPDiscrete.h"

using namespace std;

ValueFunctionDecPOMDPDiscrete::ValueFunctionDecPOMDPDiscrete(
    PlanningUnitDecPOMDPDiscrete& p , JointPolicyDiscretePure& jp):
    _m_planningUnitDecPOMDPDiscrete(&p), //BAS is never used, _m_pu used instead
    _m_jointPolicyDiscretePure(&jp)
{
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
    _m_cache.clear();
}

void ValueFunctionDecPOMDPDiscrete::SetCached(Index sI, Index JOHI, double val)
{
    _m_cache[GetCombinedIndex(sI, JOHI)]=val;
}

double ValueFunctionDecPOMDPDiscrete::GetCached(Index sI, Index JOHI,bool &valid)
{
    CacheType::const_iterator it=_m_cache.find(GetCombinedIndex(sI, JOHI));
    valid=(it!=_m_cache.end());
    if (valid)
        return it->second;
    else
        return 0;
}

template <bool CACHED>
double ValueFunctionDecPOMDPDiscrete::CalculateV()
{
#if DEBUG_CALCV
    if(DEBUG_CALCV_CACHE)
        std::cout << "CalculateV0Recursively::creating a new value function" << std::endl;
    std::cout << "evaluating joint policy:\n"; GetJPol()->Print();
#endif

    double val = 0;
    for(Index sI = 0; sI < _m_nrS; sI++)
    {
        double v_sI=CalculateVsjohRecursively<CACHED>(sI, Globals::INITIAL_JOHI, 0);
#if DEBUG_CALCV
        std::cout << ">>>ValueFunctionDecPOMDPDiscrete::CalculateV() -"
                  << " CalculateVsjohRecursively(sI=" << sI
                  << ", INITIAL_JOHI, Cached ) = " << v_sI << std::endl;
#endif
        val +=  _m_pu->GetInitialStateProbability(sI) * v_sI;
    }
#if DEBUG_CALCV
    std::cout << "This policy's V=" << val <<std::endl;
#endif
    return val;
}

template <bool CACHED>
double ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively(Index sI,Index johI, Index stage)
{
#if DEBUG_CALCV
    std::cout<< "\nValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
             << sI << ", " << johI << ") called"<<std::endl;
#endif

    Index jaI = GetJPol()->GetJointActionIndex(johI);
    double R = _m_pu->GetReward(sI, jaI);
    double ExpFutureR = 0.0;
    // horizon h policy makes decision on observation histories which 
    // maximally have length h-1. 
    // t=0 - () - length=0 
    // ... 
    // t=h-1 - (o1,...,o{h-1}) - length h-1
    if( ( CACHED && stage >= _m_h - 1) ||
        (!CACHED && _m_pu->GetTimeStepForJOHI(johI) >= _m_h - 1 ) )
    {
#if DEBUG_CALCV
        std::cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjoh"
                  << "Recursively("<< sI <<", " << johI << ") - V="<<R<<std::endl;
#endif
        return(R);
    }

#if DEBUG_CALCV
    std::cout << "Calculating future reward"<<std::endl;
#endif
    for(Index sucSI = 0; sucSI < _m_nrS; sucSI++)
    {
        double probSucSI = _m_pu->GetTransitionProbability(sI, jaI,sucSI);
#if DEBUG_CALCV
        std::cout << "P(s"<<sucSI<<"|s"<<sI<<",ja"<<jaI<<")="<<probSucSI<<std::endl;
#endif

        for(Index joI = 0; joI < _m_nrJO; joI++)
        {
            double probJOI =  _m_pu->GetObservationProbability(jaI, sucSI, joI);
#if DEBUG_CALCV
            std::cout << "P(jo"<<joI<<"|ja"<<jaI<<",s"<<sucSI<<")="<<probJOI<<std::endl;
#endif
            Index sucJohI = _m_pu->GetSuccessorJOHI(johI, joI);

            double thisSucV;
            bool valid=false;
            if (CACHED)
                thisSucV=GetCached(sucSI,sucJohI,valid); // read from cache
            if (!valid) // if not found in cache or cache is not used
                thisSucV=CalculateVsjohRecursively<CACHED>(sucSI,sucJohI,stage+1);
            ExpFutureR += probSucSI * probJOI * thisSucV;
        }//end for each observation
    }//end for each potential succesor state
    double val = R + ExpFutureR;
    if (CACHED)
        SetCached(sI, johI, val); // write to cache
#if DEBUG_CALCV
    std::cout << "ValueFunctionDecPOMDPDiscrete::CalculateVsjohRecursively("
              << sI <<", " << johI << ") \n->immediate R="<<R
              << " \n->exp. future reward="<<ExpFutureR<<"\n->V="<<val<<std::endl;
#endif
    return(val);
}

// instantiate both template options (true and false) so we don't  
// have to move the method definition to header file
template double ValueFunctionDecPOMDPDiscrete::CalculateV<true>();
template double ValueFunctionDecPOMDPDiscrete::CalculateV<false>();
