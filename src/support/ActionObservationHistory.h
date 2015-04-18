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
#ifndef _ACTIONOBSERVATIONHISTORY_H_
#define _ACTIONOBSERVATIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "IndividualHistory.h"

//forward declation:
class PlanningUnitMADPDiscrete;



/// ActionObservationHistory represents an action-observation history of an agent.
class ActionObservationHistory  : public Referrer<PlanningUnitMADPDiscrete>,
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
