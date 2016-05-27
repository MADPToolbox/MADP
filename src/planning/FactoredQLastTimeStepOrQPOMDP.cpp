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

#include "FactoredQLastTimeStepOrQPOMDP.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "QPOMDP.h"

FactoredQLastTimeStepOrQPOMDP::
FactoredQLastTimeStepOrQPOMDP(const PlanningUnitFactoredDecPOMDPDiscrete* puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QPOMDP=new QPOMDP(puf);
}
FactoredQLastTimeStepOrQPOMDP::
FactoredQLastTimeStepOrQPOMDP(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QPOMDP=new QPOMDP(puf);
}

//Destructor
FactoredQLastTimeStepOrQPOMDP::~FactoredQLastTimeStepOrQPOMDP()
{
    delete(_m_QPOMDP);
}

double FactoredQLastTimeStepOrQPOMDP::GetQ(Index jaohI, Index jaI) const
{
    // do some checking about the timestep?

    return(_m_QPOMDP->GetQ(jaohI,jaI));
}

void FactoredQLastTimeStepOrQPOMDP::DeInitialize()
{
    FactoredQLastTimeStepOrElse::DeInitialize();

    _m_initialized=false;
}

void FactoredQLastTimeStepOrQPOMDP::Initialize()
{
    FactoredQLastTimeStepOrElse::Initialize();

    _m_initialized = true;
}

void FactoredQLastTimeStepOrQPOMDP::Compute()
{
    if(!_m_initialized)
        Initialize();

    FactoredQLastTimeStepOrElse::Compute();

    _m_QPOMDP->Compute();
}
