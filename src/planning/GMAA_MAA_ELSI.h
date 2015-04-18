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
#ifndef _GMAA_MAA_ELSI_H_
#define _GMAA_MAA_ELSI_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete.h"
#include "PolicyPoolJPolValPair.h"

class BayesianGameIdenticalPayoff;


/**\brief GMAA_MAA_ELSI Generalized MAA* Exploiting Last-Stage Independence
 *
 * is a class that represents a planner that performs
 * MAA* as described by Szer et al. (see #refMAA in DOC-References.h) for 
 * factored DecPOMDPs (FactoredDecPOMDPDiscreteInterface)
 * 
 * MAA* performs a heuristic search for the optimal policy. When used 
 * with an admissible heuristic it is guaranteed to find the optimal policy.
 *
 * GMAA_MAA_ELSI exploits locality of interaction in the last stage:
 * rather than constructing a BG and enumerating all joint BG policies, 
 * it constructs a CGBG, and solves it with non-serial dynamic programming (NDP)
 *
 *
 * \sa refMAA 
 * */
class GMAA_MAA_ELSI 
    : public GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete
{
    private:

    protected:        
        /**\brief the (main part of the) 'NEXT' function from #refGMAA.
         *
         * The function that from a given partial policy from the policy
         * pool constructs a new set of (partial) joint policies.
         *
         * This function can be overrides the one in the base class
         * GeneralizedMAAStarPlannerForDecPOMDPDiscrete
         *
         * \sa GeneralizedMAAStarPlanner 
         **/
        bool ConstructAndValuateNextPolicies(
            const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi,
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
            bool &cleanUpPPI);
        void SelectPoliciesToProcessFurther(
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
            bool are_LBs, double bestLB);
        bool CAVNP_quick_n_dirty2(
            const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi, 
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies);

        void ResetPlanner();
    public:
        
        // Constructor, destructor and copy assignment.
        /**\brief Constructor.
         *
         * Takes the planning horizon as argument and
         * a pointer to the FactoredDecPOMDPDiscreteInterface for which planning
         * takes place.  the booleans arguments are passed down to
         * PlanningUnitMADPDiscrete.  .*/
        GMAA_MAA_ELSI(const PlanningUnitMADPDiscreteParameters &params, size_t
                horizon=3, FactoredDecPOMDPDiscreteInterface* p=0);

        /**\brief Default constructor.
         */
        GMAA_MAA_ELSI(size_t horizon=3, FactoredDecPOMDPDiscreteInterface* p=0);

        
//auxil. functions check which are necessary...
//

        /**Fills the (empty) vector firstOHtsI, with the indices (for each 
         * agent) of the first observation history of time step ts.*/
        void  Fill_FirstOHtsI(Index ts, std::vector<Index>& firstOHtsI);
        /**Fills the array of joint observation given the individual types and
         * offsets (firstOHtsI).*/
        void Fill_joI_Array(const Index ts, const std::vector<Index>& indTypes, 
                const std::vector<Index>& firstOHtsI, Index* joI_arr);
        /**Gets the joint observation history from joI_Array.*/
        const JointObservationHistoryTree* Get_joht(const Index ts, 
                const Index* joI_arr);
        /**Fills the array jaI_arr with the joint actions taken for joht 
         * according to jpolPrevTs.*/
        void Fill_jaI_Array(Index ts, const JointObservationHistoryTree* joht, 
                            const boost::shared_ptr<const PartialJointPolicyDiscretePure> &jpolPrevTs,
                            Index* jaI_arr);
        /**Calculates the jaohI corresponding to jaI_arr and joI_arr and also 
         * returnes the P(jaohI) and the expected obtained reward for previous
         * time steps GIVEN this joint action history.
         *
         * input args
         *  Index ts, Index jtI, Index* jaI_arr,Index* joI_arr, 
         * output args 
         *  Index& jaohI, double& PjaohI, double& ExpR_0_prevTS_thisJAOH 
         *  
         * */
        JointBeliefInterface* ProbRewardForjoahI(
                Index ts, Index jtI, Index* jaI_arr, Index* joI_arr, 
                Index& jaohI, double& PjaohI, double& ExpR_0_prevTS_thisJAOH );

        bool ConstructAndValuateNextPoliciesExactBG(
                const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi, 
                const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies);
        /**\brief construct a BG for stage t given \phi^t.
         *
         * jpolPrevTs is a partial pure joint policy that specifies actions up
         * to stage t. Given this `past' policy, this function constructs a BG
         * for stage t.
         *
         * Additionally returned are 
         *  nrOHts      -   a vector containing the number of obs-histories for
         *                  of stage t for each agent.
         *  nrJOHts     -   the number of joint observation histories for stage t
         *  firstOHtsI  -   the indices of the first OH of stage t for each agent
         *  ExpR_0_prevTS - the expected reward for stages 0...(t-1) given
         *                  jpolPrevTs.
         */
        BayesianGameIdenticalPayoff * ConstructBayesianGame(
            const boost::shared_ptr<const PartialJointPolicyDiscretePure> &jpolPrevTs,
            std::vector<size_t>& nrOHts,
            size_t& nrJOHts,
            std::vector<Index>& firstOHtsI,
            double &ExpR_0_prevTS     
        ) ;



};


#endif /* !_GMAA_MAA_ELSI_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
