/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
