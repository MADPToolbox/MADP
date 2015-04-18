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
#ifndef _ACTIONOBSERVATIONHISTORYTREE_H_
#define _ACTIONOBSERVATIONHISTORYTREE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "TreeNode.h"
#include "ActionObservationHistory.h"


/// ActionObservationHistoryTree is a wrapper for ActionObservationHistory.
/**
 * A class derived from #TreeNode, and similar to #ObservationHistoryTree:
 *
 * ActionObservationHistoryTree is a class that represents a wrapper for the 
 * ActionObservationHistory class. An ActionObservationHistoryTree actually 
 * represents  a node in the tree of observation histories. But each node also 
 * specifies a (sub-)tree so there is no actual difference between a tree and 
 * a node. 
 * This implementation assumes that ActionObservationHistories are always 
 * contained in exactly 1 ActionObservationHistoryTree: i.e., deleting an 
 * object of ActionObservationHistoryTree will free the memory of the node and 
 * the subtree represented by it as well as the memory of all the contained 
 * ActionObservationHistories.
 *
 * A difference with #ObservationHistoryTree is that here a successor is 
 * specified by 2 indices. One option would be to combine these into a joint
 * index, but this would add calculation every time we're traversing the tree.
 * Instead this class defines 2 types of nodes: ones that specify the 
 * action successor (A_SUC) and ones that specify the observation successor
 * (O_SUC).
 * Only the A_SUC nodes contain actual ActionObservationHistories, O_SUC nodes
 * are 'intermediate nodes' and no operations should be performed on them.
 * */
class ActionObservationHistoryTree :  public TreeNode <ActionObservationHistory>
{
    private:
        enum aoh_t {A_SUC, O_SUC};

        aoh_t _m_nodeType;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        ActionObservationHistoryTree(aoh_t nt = A_SUC) :
            TreeNode<ActionObservationHistory>(),
            _m_nodeType(nt){};

        /// Create a joint observation history tree for joh
        ActionObservationHistoryTree(ActionObservationHistory *const aoh, 
                aoh_t nt = A_SUC);

        /// Copy constructor.
        ActionObservationHistoryTree(const ActionObservationHistoryTree& a);
        //operators:

        //data manipulation (set) functions:
        /// Sets the index to i.
        void SetIndex(Index i);
        /// Sets the sucI'th successor of this TreeNode to suc.
        /** (e.g. the successor for observation number sucI). */
        void SetSuccessor(Index aI, Index oI,ActionObservationHistoryTree* suc);

        
        //get (data) functions:
        /// Get the successor node.
        ActionObservationHistoryTree* GetSuccessor(Index aI, Index oI);
        /// Get the history stored in this node.
        ActionObservationHistory* GetActionObservationHistory() const;

        void Print() const;
};


#endif /* !_ACTIONOBSERVATIONHISTORYTREE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
