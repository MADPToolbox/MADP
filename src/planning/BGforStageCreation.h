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
#ifndef _BGFORSTAGECREATION_H_
#define _BGFORSTAGECREATION_H_ 1

/* the include directives */
#include "Globals.h"
class PlanningUnitMADPDiscrete;
class JointPolicyDiscretePure;

/** \brief BGforStageCreation is a class that provides some functions to
 * aid the construction of Bayesian games for a stage of a Dec-POMDP.
 *
 * this is a dummy class that only contains static functions.
 * (but by inheriting from this, derived classes can use these functions)
 *
 * */
class BGforStageCreation 
{
    private:    
    
    protected:
    
        /**\brief Extends a previous policy jpolPrevTs to the next stage.
         *
         * This function extends a previous policy jpolPrevTs for ts-1 with the 
         * behavior specified by the policy of the BayesianGame for time step ts
         * (jpolBG).
         * jpolPrevTs - a joint policy for the DecPOMDP up to time step ts-1
         *              (i.e. with depth=ts-2)
         * jpolBG     - a joint policy for the BayesianGame for time step ts.
         * nrOHts     - a vector that specifies the number of observation 
         *              histories
         *              for eac agents at time step ts.
         * firstOHtsI - a vector that specifies the index of the first time step
         *              ts observation history for each agent (this functions
         *              as the offset in the conversion BG->DecPOMDP index 
         *              conversion).
         *
         * returns a new JointPolicyDiscretePure (so it must be explicitly 
         * deleted)
        PartialJointPolicyDiscretePure* ConstructExtendedPolicy(
                PartialJointPolicyDiscretePure & jpolPrevTs 
                , JointPolicyDiscretePure& jpolBG
                , std::vector<size_t>& nrOHts
                , std::vector<Index>& firstOHtsI);
         * */

        /**Fills the (empty) vector firstOHtsI, with the indices (for each 
         * agent) of the first observation history of time step ts.*/
        void  Fill_FirstOHtsI(const PlanningUnitMADPDiscrete* pu, 
                Index ts, std::vector<Index>& firstOHtsI);
        /**Fills the array of joint observation given the individual types and
         * offsets (firstOHtsI).*/
        void Fill_joI_Array(const PlanningUnitMADPDiscrete* pu, 
                const Index ts, const std::vector<Index>& indTypes, 
                const std::vector<Index>& firstOHtsI, Index* joI_arr);
        /**Gets the joint observation history from joI_Array.*/
        //const JointObservationHistoryTree* Get_joht(const Index ts, 
                //const Index* joI_arr);
        /**Fills the array jaI_arr with the joint actions taken for the
         * JOHs as specified by the array of joint observations joIs
         * according to jpolPrevTs.*/
        void Fill_jaI_Array(const PlanningUnitMADPDiscrete* pu, 
                Index ts, Index joIs[], 
                const boost::shared_ptr<const JointPolicyDiscretePure> &jpolPrevTs, Index* jaI_arr);

    public:
/*        
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BGforStageCreation();
        /// Copy constructor.
        BGforStageCreation(const BGforStageCreation& a);
        /// Copy assignment operator
        BGforStageCreation& operator= (const BGforStageCreation& o);
*/        
        /// Destructor.
        virtual ~BGforStageCreation(){};

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_BGFORSTAGECREATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
