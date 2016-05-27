/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BELIEFSET_H_
#define _BELIEFSET_H_ 1

#include <vector>
#include "JointBeliefInterface.h"

/// Represents a belief set.
typedef std::vector<JointBeliefInterface*> BeliefSet;

#endif /* !_BELIEFSET_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
