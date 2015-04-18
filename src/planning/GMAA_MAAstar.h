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
#ifndef _GMAA_MAASTAR_H_
#define _GMAA_MAASTAR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "PartialJointPolicyPureVector.h"
#include "BGIP_SolverCreatorInterface_T.h"

/**\brief GMAA_MAAstar is a class that represents a planner that performs
 * MAA* as described by Szer et al. (see #refMAA in DOC-References.h)
 * 
 * MAA* performs a heuristic search for the optimal policy. When used 
 * with an admissible heuristic it is guaranteed to find the optimal policy.
 *
 * There are a number of differences with GMAA_kGMAA:
 *    -this class does perform incremental expansion, while kGMAA does not.
 *    -this class asserts that the BG solver is exact.
 *    -
 *
 * \sa refMAA 
 * */
class GMAA_MAAstar : public GeneralizedMAAStarPlannerForDecPOMDPDiscrete
{
    private:

        /**\brief The BGIP_SolverCreator object is used to creates a BGIP_Solver of 
         * the correct type.
         *
         * Also it already contains the correct parameters (those are set when
         * the BGIP_SolverCreator was initialized).
         */
        const BGIP_SolverCreatorInterface/*_T<JointPolicyPureVector>*/ * _m_newBGIP_Solver;

    protected:        
        /**\brief the (main part of the) 'NEXT' function from #refGMAA.
         *
         * The function that from a given partial policy from the policy
         * pool constructs a new set of (partial) joint policies.
         *
         * This function can be overrides the one in the base class
         * GeneralizedMAAStarPlannerForDecPOMDPDiscrete
         *
         * \sa GeneralizedMAAStarPlannerForDecPOMDPDiscrete 
         **/
        bool ConstructAndValuateNextPolicies(
            const PartialPolicyPoolItemInterface_sharedPtr &ppi,
            const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies, 
            bool &cleanUpPPI);
        /**\brief the last part of the 'NEXT'  function from #refGMAA.
         *
         * This filters out some policies we do not want to process further.
         * Typically this function is used to only return the best k
         * policies.
         *
         * Also, this function returns nothing if are_LBs==true, which 
         * indicates that ALL the policies in poolOfNextPolicies are 
         * full-length policies (that should not be processed further).
         *
         * This function can be overrides the one in the base class*/
        void SelectPoliciesToProcessFurther(
            const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
            bool are_LBs, double bestLB);
   
        /**\brief Return a new PartialPolicyPoolInterface*.
         *
         * This function returns a pointer to new instance of the 
         * PartialPolicyPoolInterface used by this class.
         *
         * In this class this is a PolicyPoolPartialJPolValPair
         */
        PartialPolicyPoolInterface_sharedPtr NewPP()
        {return PartialPolicyPoolInterface_sharedPtr(new PolicyPoolPartialJPolValPair);};
        /**\brief Return a new PartialPolicyPoolItemInterface*.
         *
         * This function returns a pointer to new instance of the 
         * PartialPolicyPoolItemInterface used by this class.
         *
         * In this class this is a JPolValPair
         */
        PartialPolicyPoolItemInterface_sharedPtr NewPPI(const PJPDP_sharedPtr &jp,
                                                        double v) const;
        
        void ResetPlanner();

    public:
        
        // Constructor, destructor and copy assignment.
        /**\brief Constructor.
         *
         * Takes the planning horizon as argument and
         * a pointer to the DecPOMDPDiscreteInterface for which planning
         * takes place. */
        GMAA_MAAstar(
                const PlanningUnitMADPDiscreteParameters &params,
                const BGIP_SolverCreatorInterface/*_T<JointPolicyPureVector>*/ * bgs,
                size_t horizon=3, 
                DecPOMDPDiscreteInterface* p=0,
                int verboseness=0
                     );


};


#endif /* !_GMAA_MAASTAR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
