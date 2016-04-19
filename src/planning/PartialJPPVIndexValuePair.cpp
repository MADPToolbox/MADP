/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PartialJPPVIndexValuePair.h"
#include "PartialJointPolicyPureVector.h"

using namespace std;

PartialJPPVIndexValuePair::PartialJPPVIndexValuePair(PartialJointPolicyPureVector* jp, double val) :
    PartialJointPolicyValuePair(val)
{
    _m_jpol=0;
    _m_jpolIndex=jp->GetIndex();
    _m_jpolDepth=jp->GetDepth();
    _m_pu=jp->JointPolicyDiscretePure::GetInterfacePTPDiscretePure();
}                

PartialJPPVIndexValuePair::PartialJPPVIndexValuePair(const PartialJointPolicyPureVector& jp,
                                   double val) :
    PartialJointPolicyValuePair(val)
{
    _m_jpol=0;
    _m_jpolIndex=jp.GetIndex();
    _m_jpolDepth=jp.GetDepth();
    _m_pastR = jp.GetPastReward();
    _m_pu=jp.JointPolicyDiscretePure::GetInterfacePTPDiscretePure();
}

PartialJPPVIndexValuePair::~PartialJPPVIndexValuePair()
{
    delete _m_jpol;
}

PartialJointPolicyPureVector* PartialJPPVIndexValuePair::GetPartialJPPV()
{
    if(_m_jpol==0)
        AllocateJPPV();

    return(_m_jpol);
}

void PartialJPPVIndexValuePair::AllocateJPPV()
{
    if(_m_jpol==0) // not yet instantiated, do it now
    {
        _m_jpol=new PartialJointPolicyPureVector(_m_pu, OHIST_INDEX,
                                                 _m_pastR, _m_jpolDepth);
//        _m_jpol->SetDepth(_m_jpolDepth);
        _m_jpol->SetIndex(_m_jpolIndex);
    }
}

string PartialJPPVIndexValuePair::SoftPrint() const
{
    stringstream ss;
    ss << "PartialJPPVIndexValuePair: val="<< GetValue() <<", jpolIndex:"; 
    ss << _m_jpolIndex;
    return(ss.str());
}

string PartialJPPVIndexValuePair::SoftPrintBrief() const
{ 
    stringstream ss;
    ss << "PartialJPPVIndexValuePair(" << GetValue() << ","
       << _m_jpolIndex << ")";
    return(ss.str());
}
