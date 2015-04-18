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
#ifndef _JOINTOBSERVATIONHISTORY_H_
#define _JOINTOBSERVATIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include "Globals.h"
#include "E.h"
#include "Referrer.h"
#include "JointHistory.h"

class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other

/// JointObservationHistory represents a joint observation history.
/** 
 * This consists of a joint observation history index, the last joint
 * observation (index) a pointer to the predecessor JOH.  and a vector
 * of indexes to the individual observation histories.  */
class JointObservationHistory : public Referrer<PlanningUnitMADPDiscrete>,
    public JointHistory
{
    private:
        
        ///The last joint observation (index).
        Index _m_jointObservationI;
        /**True if the last joint observation (index) is empty (i.e., there is 
         * no last joint observation.) This is particularly true for a 
         * observation history at time step t=0 in a MADP that does not issue 
         * an initial observation. (this behavior is defined in 
         * MADPComponentDiscreteObservations)*/
        bool _m_containsEmptyJOI;
        /**The predecessor joint belief. Together with the last joint 
         * observation(_m_jointObservationI) this gives a full description of 
         * this joint observation history.*/    
        JointObservationHistory* _m_pred;
        /**A vector of ints which are the indices of individual 
         * ObservationHistories this gives an alternate description of the 
         * current JointObservation History. */
        std::vector<Index> _m_individualObservationHistories;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /**Constructor - creates a new initial (=empty) JointObservationHistory.
         * This method relies on the fact that the empty individual
         * observation histories are indexed 0. */
        JointObservationHistory(PlanningUnitMADPDiscrete& pu);
        /**Creates a observation history specifying jObsI for the last joint
          * observation and pred as the preceeding JointObservationHistory.*/
        JointObservationHistory(Index jObsI, JointObservationHistory * pred);
        /// Copy constructor.
        JointObservationHistory(const JointObservationHistory& a);
        /// Destructor.
        ~JointObservationHistory();

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        ///Returns the indices of the indiv. observation histories.
        const std::vector<Index>& GetIndividualObservationHistoryIndices() const;
        ///Returns the index of the last received joint observation.
        Index GetJointObservationIndex() const
            {return(_m_jointObservationI);}


        //other
        /// Returns a pointer to a copy of this class.
        virtual JointObservationHistory* Clone() const
        { return new JointObservationHistory(*this); }

        ///SoftPrints the joint observation history.
        std::string SoftPrint() const;
        ///Prints the joint observation history.
        void Print() const {std::cout << SoftPrint(); }

};


#endif /* !_OBSERVATIONHISTORY_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
