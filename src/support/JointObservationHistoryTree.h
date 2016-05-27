/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTOBSERVATIONHISTORYTREE_H_
#define _JOINTOBSERVATIONHISTORYTREE_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include "Globals.h"
#include "JointObservationHistory.h"
#include "TreeNode.h"
//#include "PlanningUnitMADPDiscrete.h"
class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other



/** 
 * \brief JointObservationHistoryTree is a class that represents a
 * wrapper for the JointObservationHistory class.
 *
 * An JointObservationHistoryTree actually represents a node in the
 * tree of observation histories. But each node also specifies a
 * (sub-)tree so there is no actual difference between a tree and a
 * node.  This implementation assumes that ObservationHistories are
 * always contained in exactly 1 JointObservationHistoryTree: i.e.,
 * deleting an object of JointObservationHistoryTree will free the
 * memory of the node and the subtree represented by it as well as the
 * memory of all the contained ObservationHistories.
 * 
*/
class JointObservationHistoryTree : public TreeNode<JointObservationHistory>
{
    private:
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.

        /// Create a joint observation history tree for joh
        JointObservationHistoryTree(JointObservationHistory *const joh) :
            TreeNode<JointObservationHistory> (joh)
        {}
        /// Copy constructor.
        JointObservationHistoryTree(const JointObservationHistoryTree& a):
            TreeNode<JointObservationHistory> (a)
        {}
        /// Destructor.
        //~JointObservationHistoryTree();

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        ///Returns the length of the contained ObservationHistory.
        size_t GetLength() const 
        {
            return( 
                (GetContainedElement()!=0)?
                GetContainedElement()->GetLength() :
                throw E("_m_jObsHist undefined!")
            );
        };
        JointObservationHistory* GetJointObservationHistory() const
        {return GetContainedElement();}
        
        JointObservationHistoryTree* GetPredecessor() const
        {return (JointObservationHistoryTree*) //we know we only put pointers
            //to JointObservationHistoryTree's in here.
            TreeNode<JointObservationHistory>::GetPredecessor();}

        JointObservationHistoryTree* GetSuccessor(Index jObsI)
        {return (JointObservationHistoryTree*) //we know we only put pointers
            //to JointObservationHistoryTree's in here.
            TreeNode<JointObservationHistory>::GetSuccessor(jObsI);}
};


#endif /* !_JOINTOBSERVATIONHISTORYTREE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
