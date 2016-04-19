/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "FactoredQLastTimeStepOrQBG.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "QBG.h"

using namespace std;

FactoredQLastTimeStepOrQBG::
FactoredQLastTimeStepOrQBG(const PlanningUnitFactoredDecPOMDPDiscrete* puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QBG=new QBG(puf);
    _m_initialized = false;
}
FactoredQLastTimeStepOrQBG::
FactoredQLastTimeStepOrQBG(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf) :
    FactoredQLastTimeStepOrElse(puf)
{
    _m_QBG=new QBG(puf);
    _m_initialized = false;
}

//Destructor
FactoredQLastTimeStepOrQBG::~FactoredQLastTimeStepOrQBG()
{
    delete(_m_QBG);
}

double FactoredQLastTimeStepOrQBG::GetQ(Index jaohI, Index jaI) const
{
    // do some checking about the timestep?

    return(_m_QBG->GetQ(jaohI,jaI));
}

void FactoredQLastTimeStepOrQBG::DeInitialize()
{
    FactoredQLastTimeStepOrElse::DeInitialize();

    _m_initialized=false;
}

void FactoredQLastTimeStepOrQBG::Initialize()
{
    cout << "FactoredQLastTimeStepOrQBG::Initialize()"<<endl;
    FactoredQLastTimeStepOrElse::Initialize();

    _m_initialized = true;
}

void FactoredQLastTimeStepOrQBG::Compute()
{
    cout << "FactoredQLastTimeStepOrQBG::Compute()"<<endl;
    if(!_m_initialized)
        Initialize();

    FactoredQLastTimeStepOrElse::Compute();

    _m_QBG->Compute();
}
