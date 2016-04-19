/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
