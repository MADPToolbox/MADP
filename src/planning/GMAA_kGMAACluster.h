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
#ifndef _GMAA_kGMAACLUSTER_H_
#define _GMAA_kGMAACLUSTER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "BayesianGameWithClusterInfo.h"
#include "JointPolicyPureVectorForClusteredBG.h"
class JointPolicyDiscretePure;
#include "BGIP_SolverCreatorInterface_T.h"
#include "BGIP_IncrementalSolverCreatorInterface_T.h"


/**\brief GMAA_kGMAA is a class that represents a GMAA planner that performs
 * k-GMAA, i.e. forward-sweep policy computation, but then returning the k
 * best-ranked policies from 'NEXT'. (see #refGMAA in DOC-References.h)
 * 
 * kGMAA computes an (approximate) policy using a (approximate) Q-value function. 
 *
 * \sa refGMAA 
 *
*/
class GMAA_kGMAACluster : public GeneralizedMAAStarPlannerForDecPOMDPDiscrete
{
    private:
        /**\brief The BGIP_SolverCreatorInterface_T object is used to creats a BGIP_Solver of 
         * the correct type.
         *
         * Also it already contains the correct parameters (those are set when
         * the BGIP_SolverCreatorInterface_T was initialized).
         */
        const BGIP_IncrementalSolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * _m_newBGIP_Solver;
        /**\brief cache for (past) joint policies.
         *
         * When returning from ConstructAndValuateNextPolicies(ppi, NextPols), 
         * GeneralizedMAAStarPlannerForDecPOMDPDiscrete will delete ppi and the contained 
         * past joint policy. However, since the used 
         * JointPolicyPureVectorForClusteredBG store backwards pointers, we
         * need to preserve these policies. _m_jpolCache is a cache that stores
         * copies of the past joint policies (to which new policies point).
         */
        std::vector< JointPolicyPureVectorForClusteredBG* > _m_jpolCache;

        /** \brief Stores the number of joint types after clustering
         * for each BG and each timestep. */
        std::vector<std::vector<int> > _m_clusteredBGsizes;

        std::string _m_clusterStatsFilename;

        void SaveClusterStats(std::string filename) const;

        BayesianGameWithClusterInfo::BGClusterAlgorithm _m_clusterAlg;
        /*double _m_thresholdJB, _m_thresholdPjaoh;*/
    
        /// Used in NewJpol()
        boost::shared_ptr<BayesianGameWithClusterInfo> _m_dummyBG;
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
            const PartialPolicyPoolInterface_sharedPtr & poolOfNextPolicies,
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
         * In this class this is a PolicyPoolJPolValPair
         */
        PartialPolicyPoolInterface_sharedPtr NewPP()
        {return PartialPolicyPoolInterface_sharedPtr(new PolicyPoolPartialJPolValPair() );};
        /**\brief Return a new PartialPolicyPoolItemInterface*.
         *
         * This function returns a pointer to new instance of the 
         * PartialPolicyPoolItemInterface used by this class.
         *
         * In this class this is a JPolValPair
         */
        PartialPolicyPoolItemInterface_sharedPtr NewPPI(
                const boost::shared_ptr<PartialJointPolicyDiscretePure> &jp, 
                double v) const;

        /**\brief Return a new Joint policy.
         */
        virtual boost::shared_ptr<PartialJointPolicyDiscretePure> NewJPol() const;
        
        void ResetPlanner();
    public:
        
        // Constructor, destructor and copy assignment.
        /**\brief Constructor.
         *
         * Takes the planning horizon as argument and
         * a pointer to the DecPOMDPDiscreteInterface for which planning
         * takes place.  the booleans arguments are passed down to
         * PlanningUnitMADPDiscrete.  .*/
        GMAA_kGMAACluster(
                const PlanningUnitMADPDiscreteParameters &params,
                //const BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgs,
                const BGIP_SolverCreatorInterface * bgs,
                size_t horizon=3, 
                DecPOMDPDiscreteInterface* p=0,
                size_t nrPoliciesToProcess=1,
                BayesianGameWithClusterInfo::BGClusterAlgorithm clusterAlg=BayesianGameWithClusterInfo::Lossless
                );

        ~GMAA_kGMAACluster();
        
        virtual GeneralizedMAAStarPlannerForDecPOMDPDiscrete* 
            GetThisFromMostDerivedPU()
        { return this; }

//        void SetTresholdJB(double threshold) { _m_thresholdJB=threshold; }
//        void SetTresholdPjaoh(double threshold) { _m_thresholdPjaoh=threshold; }

        std::string SoftPrintClusteringStats() const;
        void PrintClusteringStats() const {
            std::cout << SoftPrintClusteringStats() << std::endl; }

        void SetClusterStatsFilename(std::string filename) {
            _m_clusterStatsFilename=filename; }

};


#endif /* !_GMAA_kGMAACLUSTER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
