/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
