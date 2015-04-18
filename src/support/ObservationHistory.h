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
#ifndef _OBSERVATIONHISTORY_H_
#define _OBSERVATIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "E.h"
#include "IndividualHistory.h"

class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other



/// ObservationHistory represents an action history of a single agent.
/** 
 * It does so by storing an index and a pointer to a preceeding
 * observation history.
 */
class ObservationHistory  : public Referrer<PlanningUnitMADPDiscrete>,
    public IndividualHistory
{
    private:
        
        ///The last observation (index).
        Index _m_observationI;

        /**True if the last observation (index) is empty (i.e., there is 
         * no last observation.) This is particularly true for a 
         * observation history at time step t=0 in a MADP that does not issue 
         * an initial observation. (this behavior is defined in 
         * MADPComponentDiscreteObservations)*/
        bool _m_containsEmptyOI;
        /**The predecessor observation hist. Together with the last joint 
         * observation(_m_observationI) this gives a full description of 
         * this observation history.*/   
        ObservationHistory* _m_pred;
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /**(default) Constructor - creates a new initial (=empty)
         * ObservationHistory for agent agentI*/
        ObservationHistory(PlanningUnitMADPDiscrete& pu, Index agentI);
        /**Creates a initial observation history specifying obsI as the
         * observation at time step t=0 (o^t=0) */
        ObservationHistory(PlanningUnitMADPDiscrete& pu, Index agentI, 
                Index obsI);
        /**Creates a observation history specifying obsI for the last 
         * observation and pred as the preceeding ObservationHistory.*/
        ObservationHistory(Index obsI, ObservationHistory* pred);
        /// Copy constructor.
        //ObservationHistory(const ObservationHistory& a);
        /// Destructor.
        ~ObservationHistory();

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        /// Check whether this history contains an empty observation.
        bool ContainsEmptyOI() const
            {return _m_containsEmptyOI;}
        ///Return a reference to the Observation history that precedes this.
        const ObservationHistory* GetPredecessor() const
            {return _m_pred;}
        ///Returns the index of the last observation.
        Index GetLastObservationIndex() const
            { return _m_observationI; }

        //other
        /// Returns a pointer to a copy of this class.
        virtual ObservationHistory* Clone() const
        { return new ObservationHistory(*this); }

        /// Prints a description of *this* to a string.
        std::string SoftPrint() const; 
        ///Print *this* to cout.
        void Print() const
        { std::cout << SoftPrint();}
};


#endif /* !_OBSERVATIONHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
