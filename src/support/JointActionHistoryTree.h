/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTACTIONHISTORYTREE_H_
#define _JOINTACTIONHISTORYTREE_H_ 1

#include "TreeNode.h"
/** \class JointActionHistoryTree
 * \brief JointActionHistoryTree is a wrapper for JointActionHistory.
 *
 * See JointObservationHistoryTree for documentation. */
class JointActionHistory;

//redefine GetJointActionHistory to be the function GetContainedElement
#define GetJointActionHistory GetContainedElement
typedef TreeNode<JointActionHistory> JointActionHistoryTree;

#endif /* !_JOINTACTIONHISTORYTREE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
