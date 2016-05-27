/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "QTreeIncPruneBG.h"
#include "TreeIncPruneBGPlanner.h"

//Default constructor
QTreeIncPruneBG::QTreeIncPruneBG(const PlanningUnitDecPOMDPDiscrete* pu)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new TreeIncPruneBGPlanner(pu);
}

QTreeIncPruneBG::QTreeIncPruneBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOH(pu)
{
    _m_p=new TreeIncPruneBGPlanner(pu);
}

//Destructor
QTreeIncPruneBG::~QTreeIncPruneBG()
{
    delete _m_p;
}

double QTreeIncPruneBG::GetQ(Index jaohI, Index jaI) const
{
    return(_m_p->GetQ(jaohI,jaI));
}

void QTreeIncPruneBG::Compute()
{
    _m_p->Plan();
}
