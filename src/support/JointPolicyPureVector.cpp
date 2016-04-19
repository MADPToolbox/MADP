/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointPolicyPureVector.h"
#include "IndexTools.h"
#include "boost/make_shared.hpp"

using namespace std;

#define DEBUG_JPPT 0 
#define DEBUG_JPPT_GETJA_LOCAL 0

JointPolicyPureVector& JointPolicyPureVector::operator= (const JointPolicyPureVector& o)
{
#if DEBUG_JPOLASSIGN 
    cout << "JointPolicyPureVector::operator=(const JointPolicyPureVector& jp) called"<<endl;
#endif
    if (this == &o) return *this;  // Gracefully handle self assignment
    // Put the normal assignment duties here...
    JointPolicyDiscretePure::operator=(o);
    JPolComponent_VectorImplementation::operator=(o);
    return *this;
}

JointPolicyPureVector& JointPolicyPureVector::operator= (const JointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVector::operator=(const JointPolicyDiscretePure& jp) called"<<endl;
#endif
    if (this == &o) return *this;   // Gracefully handle self assignment
    const JointPolicyPureVector& p = 
        dynamic_cast<const JointPolicyPureVector&>( o );
    return operator=(p);
}


string JointPolicyPureVector::SoftPrint(void) const
{
    stringstream ss;
    ss << "JointPolicyPureVector: " <<endl;
    ss << JPolComponent_VectorImplementation::SoftPrint();
    return(ss.str());
} 

string JointPolicyPureVector::SoftPrintBrief(void) const
{
    stringstream ss; 
    ss << "JPPV: ";
    ss << JPolComponent_VectorImplementation::SoftPrintBrief();
    return ss.str();
}
        

JPPV_sharedPtr JointPolicyPureVector::ToJointPolicyPureVector() const
{ 
    return boost::make_shared<JointPolicyPureVector>(*this);
}
