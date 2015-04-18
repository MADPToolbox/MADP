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
#ifndef _OBSERVATIONHISTORYTREE_H_
#define _OBSERVATIONHISTORYTREE_H_ 1

/* the include directives */
#include "ObservationHistory.h"
#include "TreeNode.h"

/// ObservationHistoryTree is a wrapper for the ObservationHistory class.
/**
 * An ObservationHistoryTree actually represents a node in the tree of
 * observation histories. But each node also specifies a (sub-)tree so
 * there is no actual difference between a tree and a node. This
 * implementation assumes that ObservationHistories are always
 * contained in exactly 1 ObservationHistoryTree: i.e., deleting an
 * object of ObservationHistoryTree will free the memory of the node
 * and the subtree represented by it as well as the memory of all the
 * contained ObservationHistories. */
class ObservationHistoryTree : public TreeNode <ObservationHistory>
{
    private:
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// Create a joint observation history tree for joh
        ObservationHistoryTree(ObservationHistory *const oh) :
            TreeNode<ObservationHistory> (oh)
        {}
        /// Copy constructor.
        ObservationHistoryTree(const ObservationHistoryTree& a):
            TreeNode<ObservationHistory> (a)
        {}

        //operators:

        //data manipulation (set) functions:
 
        //get (data) functions:
        /// Returns the length of the contained ObservationHistory.
        size_t GetLength() const  
        {
            return( 
                (GetContainedElement()!=0)?
                GetContainedElement()->GetLength() :
                throw E("_m_jObsHist undefined!")
            );
        };
        ObservationHistory* GetObservationHistory() const
        {return GetContainedElement();}

        ObservationHistoryTree* GetPredecessor() const
        {return (ObservationHistoryTree*) //we know we only put pointers
            //to ObservationHistoryTree's in here.
            TreeNode<ObservationHistory>::GetPredecessor();}

        ObservationHistoryTree* GetSuccessor(Index jObsI)
        {return (ObservationHistoryTree*) //we know we only put pointers
            //to ObservationHistoryTree's in here.
            TreeNode<ObservationHistory>::GetSuccessor(jObsI);}

};


#endif /* !_OBSERVATIONHISTORYTREE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
