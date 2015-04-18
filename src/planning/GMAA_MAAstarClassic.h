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
#ifndef _GMAA_MAASTARCLASSIC_H_
#define _GMAA_MAASTARCLASSIC_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "PartialJointPolicyPureVector.h"

/**\brief GMAA_MAAstarClassic is a class that represents a planner that performs
 * MAA* as described by Szer et al. (see #refMAA in DOC-References.h)
 * 
 * MAA* performs a heuristic search for the optimal policy. When used 
 * with an admissible heuristic it is guaranteed to find the optimal policy.
 *
 * This is the "classic" version, which means it has its own builtin
 * brute-force BGIP solver, and does not accept an arbitrary optimal
 * BGIP solver.
 *
 *  \sa refMAA */
class GMAA_MAAstarClassic : public GeneralizedMAAStarPlannerForDecPOMDPDiscrete
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
         * \sa GeneralizedMAAStarPlannerForDecPOMDPDiscrete 
         **/
        bool ConstructAndValuateNextPolicies(
            const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi,
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
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
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
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
        GMAA_MAAstarClassic(
                const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon=3, 
                DecPOMDPDiscreteInterface* p=0,
                int verboseness=0
                     );

        /**\brief Default constructor.
         */
        GMAA_MAAstarClassic(size_t horizon=3, DecPOMDPDiscreteInterface* p=0);

};


#endif /* !_GMAA_MAASTARCLASSIC_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
