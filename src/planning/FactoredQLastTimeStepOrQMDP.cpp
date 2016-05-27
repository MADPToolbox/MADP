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

#include "FactoredQLastTimeStepOrQMDP.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "QMDP.h"

FactoredQLastTimeStepOrQMDP::
FactoredQLastTimeStepOrQMDP(const PlanningUnitFactoredDecPOMDPDiscrete* puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QMDP=new QMDP( puf );
}
FactoredQLastTimeStepOrQMDP::
FactoredQLastTimeStepOrQMDP(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QMDP=new QMDP( puf );
}

//Destructor
FactoredQLastTimeStepOrQMDP::~FactoredQLastTimeStepOrQMDP()
{
    delete(_m_QMDP);
}

double FactoredQLastTimeStepOrQMDP::GetQ(Index jaohI, Index jaI) const
{
    // do some checking about the timestep?

    return(_m_QMDP->GetQ(jaohI,jaI));
}

void FactoredQLastTimeStepOrQMDP::DeInitialize()
{
    FactoredQLastTimeStepOrElse::DeInitialize();

    _m_initialized=false;
}

void FactoredQLastTimeStepOrQMDP::Initialize()
{
    FactoredQLastTimeStepOrElse::Initialize();

    _m_initialized = true;
}

void FactoredQLastTimeStepOrQMDP::Compute()
{
    if(!_m_initialized)
        Initialize();

    FactoredQLastTimeStepOrElse::Compute();

    _m_QMDP->Compute();
}
