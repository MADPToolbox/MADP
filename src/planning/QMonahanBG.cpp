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

#include "QMonahanBG.h"
#include "MonahanBGPlanner.h"

//Default constructor
QMonahanBG::QMonahanBG(const PlanningUnitDecPOMDPDiscrete* pu,
                       bool doIncPrune)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new MonahanBGPlanner(pu,doIncPrune);
}

QMonahanBG::QMonahanBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                       bool doIncPrune)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new MonahanBGPlanner(pu,doIncPrune);
}

//Destructor
QMonahanBG::~QMonahanBG()
{
    delete _m_p;
}

double QMonahanBG::GetQ(Index jaohI, Index jaI) const
{
    return(_m_p->GetQ(jaohI,jaI));
}

void QMonahanBG::Compute()
{
    _m_p->Plan();
}
