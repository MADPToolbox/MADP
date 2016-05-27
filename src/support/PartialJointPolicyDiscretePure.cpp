/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PartialJointPolicyDiscretePure.h"

using namespace std;

//Copy assignment operator
PartialJointPolicyDiscretePure& PartialJointPolicyDiscretePure::operator= (const PartialJointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "PartialJointPolicyDiscretePure& PartialJointPolicyDiscretePure::operator= (const PartialJointPolicyDiscretePure& o) called"<<endl;
#endif

    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    JointPolicyDiscretePure::operator= ( o );
    PartialJointPolicy::operator= ( o );

    return *this;
}
