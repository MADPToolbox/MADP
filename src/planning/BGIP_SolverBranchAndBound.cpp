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
