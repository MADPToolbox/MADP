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

#include "JointPolicyDiscretePure.h"
using namespace std;

//Default constructor
JointPolicyDiscretePure::JointPolicyDiscretePure(
        const Interface_ProblemToPolicyDiscretePure* pu,
        PolicyGlobals::PolicyDomainCategory idc ) :
    JointPolicyDiscrete( pu , idc )
{
    
}
JointPolicyDiscretePure::JointPolicyDiscretePure(
        const I_PtPDpure_constPtr &pu,
        PolicyGlobals::PolicyDomainCategory idc ) :
    JointPolicyDiscrete( pu , idc )
{
    
}
//Copy constructor.    
JointPolicyDiscretePure::JointPolicyDiscretePure(const 
        JointPolicyDiscretePure& o) 
    :
    JointPolicyDiscrete( o )

{
}
//Destructor
//JointPolicyDiscretePure::~JointPolicyDiscretePure()
//{
//}
//Copy assignment operator
JointPolicyDiscretePure& JointPolicyDiscretePure::operator= (const JointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyDiscretePure::operator=(const JointPolicyDiscretePure& jp) called"<<endl;
#endif
    if (this == &o) return *this;   // Gracefully handle self assignment
    JointPolicyDiscrete::operator= ( o );   //call parent
  // Put the normal assignment duties here...

    return *this;
}

double JointPolicyDiscretePure::GetJointActionProb( 
        Index i, Index ja ) const
{
    return (ja == GetJointActionIndex(i));
    
}
double JointPolicyDiscretePure::GetJointActionProb( 
        LIndex i, Index ja ) const
{
    return (ja == GetJointActionIndex(i));
    
}
double JointPolicyDiscretePure::GetActionProb( Index agentI,
        Index i, Index a ) const
{
    return (a == GetActionIndex(agentI, i));    
}

void JointPolicyDiscretePure::SetInterfacePTPDiscretePure(
        Interface_ProblemToPolicyDiscretePure* pu)
{
    SetInterfacePTPDiscrete(pu);
}

