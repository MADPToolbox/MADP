/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ACTIONOBSERVATIONHISTORY_H_
#define _ACTIONOBSERVATIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "IndividualHistory.h"

//forward declation:
class PlanningUnitMADPDiscrete;



/// ActionObservationHistory represents an action-observation history of an agent.
class ActionObservationHistory  :
    public IndividualHistory
{
    private:    
        
        ///The index of the observation history contained.
        Index _m_ohI;
        ///The index of the action history contained.
        Index _m_ahI;

        /**The predecessor together with the last joint actionObservation
         * (_m_actionObservationI) this gives a full description of 
         * this joint actionObservation history.*/   
        ActionObservationHistory* _m_pred;

        PlanningUnitMADPDiscrete* _m_planningUnitMADPDiscrete;
   
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /** (default) Constructor - creates a new initial (=empty)
         * ObservationHistory for agent agentI*/
        ActionObservationHistory(PlanningUnitMADPDiscrete& pu, Index agentI);
        /**Creates the action-observation history resulting from action aI and
         * obs. oI after ActionObservationHistory pred.*/
        ActionObservationHistory(Index aI, Index oI, 
                ActionObservationHistory* pred);

        //operators:

        //data manipulation (set) functions:
        ///Sets the action history index corresponding to this ActObsHist
        void SetActionHistoryIndex(Index ahI)
            {_m_ahI = ahI;}
        ///Sets the observation history index corresponding to this ActObsHist
        void SetObservationHistoryIndex(Index ohI)
            {_m_ohI = ohI;}
        
        //get (data) functions:      

        ///Gets the action history index corresponding to this ActObsHist
        Index GetActionHistoryIndex() const
            {return _m_ahI;}
        ///Gets the observation history index corresponding to this ActObsHist
        Index GetObservationHistoryIndex() const
            {return _m_ohI;}
        //other
        /// Returns a pointer to a copy of this class.
        virtual ActionObservationHistory* Clone() const
        { return new ActionObservationHistory(*this); }

        std::string SoftPrint() const; 
        void Print() const { std::cout << SoftPrint();};

};


#endif /* !_ACTIONOBSERVATIONHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
