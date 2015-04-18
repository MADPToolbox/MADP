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

#include "QMonahanPOMDP.h"
#include "MonahanPOMDPPlanner.h"

//Default constructor
QMonahanPOMDP::QMonahanPOMDP(const PlanningUnitDecPOMDPDiscrete* pu,
                             bool doIncPrune)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new MonahanPOMDPPlanner(pu,doIncPrune);
}
QMonahanPOMDP::QMonahanPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                             bool doIncPrune)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new MonahanPOMDPPlanner(pu,doIncPrune);
}

//Destructor
QMonahanPOMDP::~QMonahanPOMDP()
{
    delete _m_p;
}

double QMonahanPOMDP::GetQ(Index jaohI, Index jaI) const
{
    return(_m_p->GetQ(jaohI,jaI));
}

void QMonahanPOMDP::Compute()
{
    _m_p->Plan();
}
