/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PartialJointPolicy.h"

using namespace std;

//Copy assignment operator
PartialJointPolicy& PartialJointPolicy::operator= (const PartialJointPolicy& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    _m_pastReward = o._m_pastReward;
    return *this;
}
