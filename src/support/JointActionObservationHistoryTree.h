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
#ifndef _JOINTACTIONOBSERVATIONHISTORYTREE_H_
#define _JOINTACTIONOBSERVATIONHISTORYTREE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "TreeNode.h"
#include "JointActionObservationHistory.h"



/**
 * \brief JointActionObservationHistoryTree is derived from #TreeNode,
 * and similar to #ObservationHistoryTree:
 *
 * JointActionObservationHistoryTree is a class that represents a wrapper for 
 * the JointActionObservationHistory class. An 
 * JointActionObservationHistoryTree actually 
 * represents  a node in the tree of observation histories. But each node also 
 * specifies a (sub-)tree so there is no actual difference between a tree and 
 * a node. 
 * This implementation assumes that ActionObservationHistories are always 
 * contained in exactly 1 JointActionObservationHistoryTree: i.e., deleting an 
 * object of JointActionObservationHistoryTree will free the memory of the node 
 * and the subtree represented by it as well as the memory of all the contained 
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
class JointActionObservationHistoryTree :  public TreeNode <JointActionObservationHistory>
{
    private:
        enum aoh_t {A_SUC, O_SUC};

        aoh_t _m_nodeType;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        JointActionObservationHistoryTree(aoh_t nt = A_SUC) :
            TreeNode<JointActionObservationHistory> (),
            _m_nodeType(nt){};

        /// Create a joint observation history tree for joh
        JointActionObservationHistoryTree(JointActionObservationHistory *const
                aoh, aoh_t nt = A_SUC);

        //operators:

        //data manipulation (set) functions:
        /// Sets the index to i.
        void SetIndex(LIndex i);
        /// Sets the sucI'th successor of this TreeNode to suc.
        /** For example, the successor for observation number sucI. */
        void SetSuccessor(Index jaI, Index joI,
                          JointActionObservationHistoryTree* suc);

        
        //get (data) functions:
        ///Returns the length of the containted JointActionObservationHistory.
        size_t GetLength() const 
            {return(GetContainedElement()->GetLength());}
        /**\brief Returns the successor
         * JointActionObservationHistoryTree* for joint action jaI and
         * joint observation joI.*/
        JointActionObservationHistoryTree* GetSuccessor(Index jaI, Index joI);
        ///Returns the predecessor JointActionObservationHistoryTree*.
        JointActionObservationHistoryTree* GetPredecessor() const;
        ///Returns the contained JointActionObservationHistory*.
        JointActionObservationHistory* GetJointActionObservationHistory() const;

        void Print() const;
};


#endif /* !_JOINTACTIONOBSERVATIONHISTORYTREE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
