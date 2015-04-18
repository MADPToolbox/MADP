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
#ifndef _ACTIONHISTORY_H_
#define _ACTIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "E.h"
#include "IndividualHistory.h"

class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other

/// ActionHistory represents an action history of a single agent.
/** 
 * It does so by storing an index and a pointer to a preceeding
 * action history.
 */
class ActionHistory  : public Referrer<PlanningUnitMADPDiscrete>,
    public IndividualHistory
{
    private:

        ///The last action (index).
        Index _m_actionI;
        /**True if the last action (index) is empty (i.e., there is 
         * no last action.) This is true for a action history at time step 0.
         */
        bool _m_isEmpty;
        /**The predecessor joint belief. Together with the last joint 
         * action(_m_actionI) this gives a full description of 
         * this joint action history.*/   
        ActionHistory* _m_pred;
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /**(default) Constructor - creates a new initial (=empty)
         * ActionHistory for agent agentI*/
        ActionHistory(PlanningUnitMADPDiscrete& pu, Index agentI);
        
        /**Creates a action history specifying aI for the last 
         * action and pred as the preceeding ActionHistory.*/
        ActionHistory(Index aI, ActionHistory* pred);

        /// Destructor.
        ~ActionHistory();

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        /// Check whether this history is empty.
        bool IsEmpty() const
            {return _m_isEmpty;}
        ///Returns the index of the last action.
        Index GetLastActionIndex() const
            { return _m_actionI; }
        //other
        /// Returns a pointer to a copy of this class.
        virtual ActionHistory* Clone() const
        { return new ActionHistory(*this); }

        /// Prints a description of *this* to a string.
        std::string SoftPrint() const; 
        ///Print *this* to cout.
        void Print() const
        { std::cout << SoftPrint();}
};


#endif /* !_ACTIONHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
