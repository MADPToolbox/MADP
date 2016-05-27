/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
