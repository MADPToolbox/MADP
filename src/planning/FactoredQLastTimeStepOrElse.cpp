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

#include "FactoredQLastTimeStepOrElse.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"

using namespace std;

//Default constructor
FactoredQLastTimeStepOrElse::
FactoredQLastTimeStepOrElse(const PlanningUnitFactoredDecPOMDPDiscrete *puf) :
    QFunctionForFactoredDecPOMDP(puf)
{
    _m_initialized = false;
}
FactoredQLastTimeStepOrElse::
FactoredQLastTimeStepOrElse(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf) :
    QFunctionForFactoredDecPOMDP(puf)
{
    _m_initialized = false;
}

//Destructor
FactoredQLastTimeStepOrElse::~FactoredQLastTimeStepOrElse()
{
}

double FactoredQLastTimeStepOrElse::GetLocalQValue(
        Index LRF, 
        Index stage,
        const vector<Index>& sfacValues,  
        const vector<Index>& aoHistIs, 
        Index agSc_jaI //joint group action index
) const
{

    const PlanningUnitFactoredDecPOMDPDiscrete*  puf = GetPUF();
    const FactoredDecPOMDPDiscreteInterface* fdpi = puf->GetFDPOMDPD();
    Index sI = fdpi->RestrictedStateVectorToJointIndex(LRF, sfacValues);
    double r = fdpi->GetLRFReward(LRF, sI, agSc_jaI);
    return(r);
};


void FactoredQLastTimeStepOrElse::DeInitialize()
{
    _m_initialized=false;
}

void FactoredQLastTimeStepOrElse::Initialize()
{
    cout << "FactoredQLastTimeStepOrElse::Initialize()"<<endl;
    _m_nrLRFs=GetPUF()->GetFDPOMDPD()->GetNrLRFs();
    _m_agentScopes.clear();
    for(Index e=0;e!=_m_nrLRFs;++e)
        _m_agentScopes.push_back(GetPUF()->GetFDPOMDPD()->GetAgentScopeForLRF(e));
    _m_initialized = true;
}

void FactoredQLastTimeStepOrElse::Compute()
{
    cout << "FactoredQLastTimeStepOrElse::Compute()"<<endl;
    if(!_m_initialized)
        Initialize();

    //compute...
}
size_t FactoredQLastTimeStepOrElse::GetNrLQFs(Index stage) const
{
    Index last_stage = GetPUF()->GetHorizon() - 1;
    if(stage != last_stage)
        throw E("FactoredQLastTimeStepOrElse::GetNrLQFs called for stage other than the last!");
    return _m_nrLRFs;
}

const Scope& FactoredQLastTimeStepOrElse::GetAgentScopeForLQF(Index e,Index stage) const
{
    Index last_stage = GetPUF()->GetHorizon() - 1;
    if(stage != last_stage)
        throw E("FactoredQLastTimeStepOrElse::GetNrLQFs called for stage other than the last!");
    return _m_agentScopes.at(e);
}

const Scope& FactoredQLastTimeStepOrElse::GetStateFactorScopeForLQF(Index e,Index stage) const
{
    Index last_stage = GetPUF()->GetHorizon() - 1;
    if(stage != last_stage)
        throw E("FactoredQLastTimeStepOrElse::GetNrLQFs called for stage other than the last!");
    return GetPUF()->GetFDPOMDPD()->GetStateFactorScopeForLRF(e);
}

