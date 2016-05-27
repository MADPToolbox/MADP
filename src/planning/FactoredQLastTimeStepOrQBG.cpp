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
