/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JPPVIndexValuePair.h"
#include "JointPolicyPureVector.h"

using namespace std;

JPPVIndexValuePair::JPPVIndexValuePair(JointPolicyPureVector* jp,
                                       double value) :
    JointPolicyValuePair(value)
{
    _m_jpol=0;
    _m_jpolIndex=jp->GetIndex();
    _m_jpolDepth=jp->GetDepth();
    _m_pu=jp->JointPolicyDiscretePure::GetInterfacePTPDiscretePure();
}                

JPPVIndexValuePair::JPPVIndexValuePair(const JointPolicyPureVector& jp,
                                       double value) :
    JointPolicyValuePair(value)
{
    _m_jpol=0;
    _m_jpolIndex=jp.GetIndex();
    _m_jpolDepth=jp.GetDepth();
    _m_pu=jp.JointPolicyDiscretePure::GetInterfacePTPDiscretePure();
}

JPPVIndexValuePair::~JPPVIndexValuePair()
{
    delete _m_jpol;
}

JointPolicyDiscretePure* JPPVIndexValuePair::GetJPol()
{ return GetJPPV(); }

JointPolicyPureVector* JPPVIndexValuePair::GetJPPV()
{
    if(_m_jpol==0)
        AllocateJPPV();

    return(_m_jpol);
}

void JPPVIndexValuePair::AllocateJPPV()
{
    if(_m_jpol==0) // not yet instantiated, do it now
    {
        _m_jpol=new JointPolicyPureVector(_m_pu);
        _m_jpol->SetDepth(_m_jpolDepth);
        _m_jpol->SetIndex(_m_jpolIndex);
    }
}

string JPPVIndexValuePair::SoftPrint() const
{
    stringstream ss;
    ss << "JPPVIndexValuePair: value="<< GetValue() <<", jpolIndex:"; 
    ss << _m_jpolIndex;
    return(ss.str());
}

string JPPVIndexValuePair::SoftPrintBrief() const
{ 
    stringstream ss;
    ss << "JPPVIndexValuePair(" << GetValue() << ","
       << _m_jpolIndex << ")";
    return(ss.str());
}
