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

#include "PartialJointPolicyPureVector.h"
#include "JointPolicyPureVector.h"
#include "boost/make_shared.hpp"

using namespace std;

PartialJointPolicyPureVector::PartialJointPolicyPureVector(
    const Interface_ProblemToPolicyDiscretePure* pu,
    PolicyGlobals::PolicyDomainCategory idc,
    double pastReward,
    size_t depth) :
    PartialJointPolicyDiscretePure(pu, idc, pastReward),
    JPolComponent_VectorImplementation(pu, idc, depth)
{
    SetDepth(depth);
}

PartialJointPolicyPureVector::PartialJointPolicyPureVector(
    const I_PtPDpure_constPtr &pu,
    PolicyGlobals::PolicyDomainCategory idc,
    double pastReward,
    size_t depth) :
    PartialJointPolicyDiscretePure(pu, idc, pastReward),
    JPolComponent_VectorImplementation(pu, idc, depth)
{
    SetDepth(depth);
}

//Copy assignment operator
PartialJointPolicyPureVector& PartialJointPolicyPureVector::operator= (const PartialJointPolicyPureVector& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "PartialJointPolicyPureVector& PartialJointPolicyPureVector::operator= (const PartialJointPolicyPureVector& o) called"<<endl;
#endif

    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    PartialJointPolicyDiscretePure::operator=(o);
    JPolComponent_VectorImplementation::operator=(o);
    return *this;
}
PartialJointPolicyPureVector& PartialJointPolicyPureVector::operator= (const 
        PartialJointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "PartialJointPolicyPureVector& PartialJointPolicyPureVector::operator= (const PartialJointPolicyDiscretePure& o) called"<<endl;
#endif

    if (this == &o) return *this;   // Gracefully handle self assignment
    const PartialJointPolicyPureVector& p = 
        dynamic_cast<const PartialJointPolicyPureVector&>( o );
    return operator=(p);

}


string PartialJointPolicyPureVector::SoftPrint() const
{
    stringstream ss; 
    ss << "PartialJointPolicyPureVector, past reward="<<GetPastReward()<<endl;
    ss << JPolComponent_VectorImplementation::SoftPrint();
    return ss.str();
}

string PartialJointPolicyPureVector::SoftPrintBrief() const
{
    stringstream ss; 
    ss << "PartialJPPV, past R="<<GetPastReward()<<", ";
    ss << JPolComponent_VectorImplementation::SoftPrintBrief();
    return ss.str();
}

JPPV_sharedPtr PartialJointPolicyPureVector::ToJointPolicyPureVector() const
{
    JointPolicyPureVector *jppv=new JointPolicyPureVector(*this,*this);
    return(JPPV_sharedPtr(jppv));
}
