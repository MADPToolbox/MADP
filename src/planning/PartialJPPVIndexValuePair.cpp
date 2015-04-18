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
