/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
