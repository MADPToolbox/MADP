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

/* Only include this header file once. */
#ifndef _ACTIONHISTORYTREE_H_
#define _ACTIONHISTORYTREE_H_ 1

#include "TreeNode.h"
/** \class ActionHistoryTree
 * \brief ActionHistoryTree is a wrapper for ActionHistory.
 *
 * See ObservationHistoryTree for documentation. */
class ActionHistory;

//redefine GetActionHistory to be the function GetContainedElement
#define GetActionHistory GetContainedElement
typedef TreeNode<ActionHistory> ActionHistoryTree;

#endif /* !_ACTIONHISTORYTREE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
