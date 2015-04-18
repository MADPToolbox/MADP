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
