/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTACTIONHISTORY_H_
#define _JOINTACTIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include <vector>

#include "Globals.h"
#include "TreeNode.h"
#include "JointHistory.h"

//#include "PlanningUnitMADPDiscrete.h"
class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other

/// JointActionHistory represents a joint action history.
/** 
 * This consists of a joint action history index, the last joint
 * action (index) a pointer to the predecessor JOH.  and a vector of
 * indexes to the individual action histories.  */
class JointActionHistory :
    public JointHistory
{
    private:
        
        ///The last joint action (index).
        Index _m_jointActionI;
        /**True if the last joint action (index) is empty (i.e., there is 
         * no last joint action.)  This is true for a action history at 
         * time step 0.
         */
        bool _m_isEmpty;
        /**The predecessor joint belief. Together with the last joint 
         * action(_m_jointActionI) this gives a full description of 
         * this joint action history.*/    
        JointActionHistory* _m_pred;
        /**A vector of ints which are the indices of individual 
         * ActionHistories this gives an alternate description of the 
         * current JointAction History. */
        std::vector<Index> _m_individualActionHistories;

        PlanningUnitMADPDiscrete* _m_planningUnitMADPDiscrete;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /**Constructor - creates a new initial (=empty) JointActionHistory.
         * This method relies on the fact that the empty individual
         * action histories are indexed 0. */
        JointActionHistory(PlanningUnitMADPDiscrete& pu);
        /**Creates a initial action history specifying obsI as the
          * action at time step t=0 (o^t=0) */
        JointActionHistory(PlanningUnitMADPDiscrete& pu, Index jObsI);
        /**Creates a action history specifying jObsI for the last joint
          * action and pred as the preceeding JointActionHistory.*/
        JointActionHistory(Index jObsI, JointActionHistory * pred);
        /// Copy constructor.
        JointActionHistory(const JointActionHistory& a);
        /// Destructor.
        ~JointActionHistory();

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        ///Returns the indices of the indiv. action histories.
        const std::vector<Index>& GetIndividualActionHistoryIndices() const;
        //other
        /// Returns a pointer to a copy of this class.
        virtual JointActionHistory* Clone() const
        { return new JointActionHistory(*this); }

        ///SoftPrints the joint observation history.
        std::string SoftPrint() const;
        ///Prints the joint observation history.
        void Print() const { std::cout << SoftPrint(); }

};

#endif /* !_JOINTACTIONHISTORY_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
