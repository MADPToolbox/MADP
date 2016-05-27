/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _GMAA_kGMAA_H_
#define _GMAA_kGMAA_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "BGIP_SolverCreatorInterface_T.h"

/**\brief GMAA_kGMAA is a class that represents a GMAA planner that performs
 * k-GMAA, i.e. forward-sweep policy computation, but then returning the k
 * best-ranked policies from 'NEXT'. (see #refGMAA in DOC-References.h)
 * 
 * kGMAA computes an (approximate) policy using a (approximate) Q-value function. 
 *
 * \sa refGMAA 
 * */
class GMAA_kGMAA : public GeneralizedMAAStarPlannerForDecPOMDPDiscrete
{
    private:
        /**\brief The BGIP_SolverCreator object is used to creates a BGIP_Solver of 
         * the correct type.
         *
         * Also it already contains the correct parameters (those are set when
         * the BGIP_SolverCreator was initialized).
         */
        //const BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * _m_newBGIP_Solver;
        const BGIP_SolverCreatorInterface * _m_newBGIP_Solver;

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
         * In this class this is a PolicyPoolJPolValPair
         */
        boost::shared_ptr<PartialPolicyPoolInterface> NewPP();
        //{return (new PolicyPoolJPolValPair);};
        /**\brief Return a new PartialPolicyPoolItemInterface*.
         *
         * This function returns a pointer to new instance of the 
         * PartialPolicyPoolItemInterface used by this class.
         *
         * In this class this is a JPolValPair
         */
        boost::shared_ptr<PartialPolicyPoolItemInterface> NewPPI(const PJPDP_sharedPtr &jp,
                                                                 double v) const;
        //{return (new JPolValPair(jp,v));};
        
        void ResetPlanner();

    public:
        
        // Constructor, destructor and copy assignment.
        /**\brief Constructor.
         *
         * Takes the planning horizon as argument and
         * a pointer to the DecPOMDPDiscreteInterface for which planning takes place.
         * the booleans arguments are passed down to PlanningUnitMADPDiscrete. 
         * */
        GMAA_kGMAA(
            const PlanningUnitMADPDiscreteParameters &params,
            //const BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * bgs,
            const BGIP_SolverCreatorInterface * bgs,
            size_t horizon=3,
            DecPOMDPDiscreteInterface* p=0,
            size_t nrPoliciesToProcess=1);

};


#endif /* !_GMAA_OPTIMALFORWARDSWEEP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
