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
#ifndef _JOINTACTIONOBSERVATIONHISTORY_H_
#define _JOINTACTIONOBSERVATIONHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "JointHistory.h"

//forward declation:
class PlanningUnitMADPDiscrete;

/// JointActionObservationHistory represents a joint action observation history.
class JointActionObservationHistory  : public Referrer<PlanningUnitMADPDiscrete>,
    public JointHistory
{
    private:    
        
        ///The last joint action (index).
        Index _m_jaI;
        ///The last joint observation (index).
        Index _m_joI;

        /**The predecessor together with the last joint actionObservation
         * (_m_actionObservationI) this gives a full description of 
         * this joint actionObservation history.*/   
        JointActionObservationHistory* _m_pred;
        
        /**A vector of Indices which are the indices of individual 
         * ActionObservationHistories this gives an alternate description of the
         * current JointActionObservation History. */
        std::vector<Index> _m_individualActionObservationHistories;

        void GetJointActionObservationHistoryVectorsRecursive(
            std::vector<Index> &jaIs, std::vector<Index> &joIs);

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /** \brief (default) Constructor - creates a new initial
         * (=empty) JointActionObservationHistory. */
        JointActionObservationHistory(PlanningUnitMADPDiscrete& pu);
        /** \brief Creates the joint action-obs. history resulting
         * from joint action jaI and obs. joI after
         * JointActionObservationHistory pred.*/
        JointActionObservationHistory(Index jaI, Index joI, 
                JointActionObservationHistory* pred);

        //operators:

        //data manipulation (set) functions:


        //get (data) functions:      
        ///Returns the indices of the indiv. action observation histories.
        const std::vector<Index>& GetIndividualActionObservationHistoryIndices()
            const
        {return _m_individualActionObservationHistories;}
        
        //other
        ///SoftPrints the history.
        std::string SoftPrint() const;
        ///SoftPrints the history in terms of joint action/observation indices.
        std::string SoftPrintJointIndices() const;
        ///Prints the history.
        void Print() const {std::cout << SoftPrint(); }
       
        /** \brief Get vectors of joint action and observation indices
         * stored by this history. */
        void GetJointActionObservationHistoryVectors(
            std::vector<Index> &jaIs, std::vector<Index> &joIs);

        ///Gets the last joint action index. 
        Index GetJointActionIndex() const
            {return _m_jaI;}
        ///Gets the last joint observation index.
        Index GetJointObservationIndex() const
            {return _m_joI;}

        /// Returns a pointer to a copy of this class.
        virtual JointActionObservationHistory* Clone() const
        { return new JointActionObservationHistory(*this); }


};


#endif /* !_JOINTACTIONOBSERVATIONHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
