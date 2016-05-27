/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "BGIP_SolverBranchAndBound.h"
#include "BGIP_BnB_Node.h"
#include <float.h>
#include <limits>

#define INITIALIZE_LB_TO_BESTFOUND 1


#define DEBUG_VALID_ACTIONS 0
#define CHECK_VALID_JA 0

using namespace std;

#if 0 
//Frans 20110922: I disabled this and moved everything into the header file, where is belongs (since this is a templated class)
#endif
