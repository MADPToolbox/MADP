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
#ifndef _BAYESIANGAMEWITHCLUSTERINFO_H_
#define _BAYESIANGAMEWITHCLUSTERINFO_H_ 1

/* the include directives */
#include "Globals.h"
#include "BayesianGameForDecPOMDPStage.h"

class JointPolicyPureVectorForClusteredBG;
class JointPolicyPureVector;
class JointPolicyDiscretePure;
#include "PartialJointPolicyDiscretePure.h"
class JointBeliefInterface;
class QFunctionJointBeliefInterface;
class TypeCluster;
typedef std::vector<TypeCluster* > TypeClusterList;
class PlanningUnitMADPDiscrete;
class QHybrid;

class BayesianGameWithClusterInfo;
typedef boost::shared_ptr<BayesianGameWithClusterInfo> BGwCI_sharedPtr;
typedef boost::shared_ptr<const BayesianGameWithClusterInfo> BGwCI_constPtr;

/** \brief BayesianGameWithClusterInfo represents an identical-payoff
 * BG that can be clustered. */
class BayesianGameWithClusterInfo : 
    public BayesianGameForDecPOMDPStage
{
public:

    enum BGClusterAlgorithm { Lossless };

    static std::string SoftPrint(BGClusterAlgorithm clusterAlg);
    private:    
        

        ///The pointer to the previous BG
        BGwCI_constPtr _m_pBG;

        /** \brief A pointer to the Qfunction, if it is defined over joint
         * beliefs, otherwise 0. */
        const QFunctionJointBeliefInterface *_m_qJB;

        /** \brief A pointer to the Qfunction, if it is a hybrid one,
         * otherwise 0. */
        const QHybrid *_m_qHybrid;

        /**\brief The pointer to the joint BG policy for the previous BG 
         * (that lead to *this* BG) 
         */
        //const JointPolicyPureVectorForClusteredBG* _m_pBGJPol;
        ///the previous *BG* policy (i.e., not a PartialJointPolicyDiscretePure)
        boost::shared_ptr<const JointPolicyDiscretePure> _m_pBGJPol;



        /**\brief A list with a representative history for each joint type.
         *
         * For each joint type we maintain a representative joint action-
         * observation history.
         */
        std::vector< Index > _m_jaohReps;

        /**storing the extra information about the individual types.
         *
         * Each agent has a list of types or TypeClusters, 
         * as they can represent
         * a cluster of action-observation histories.
         */
        std::vector< TypeClusterList* > _m_typeLists;

        /// The type of clustering we are performing.
        BGClusterAlgorithm _m_clusterAlgorithm;
        
    
    protected:
        /**\brief shifts all the probability mass from agent agI's type t2 
         * to t1.
         *
         * This function is used when clustering types t1 and t2. In 
         * particular, this function shifts the probability mass of all joint
         * types involving t2 to the corresponding ones involving t1. E.g.:
         * Let <t u> denote a joint type, then this function performs:
         * P( <t1 u> ) := P ( <t1 u> ) + P( <t2 u > )
         * P( <t2 u> ) := 0
         *
         * Note: when performing approximate clustering, this function should 
         * also average the utilities of the corresponding joint types.
         */
        void ShiftProbabilityAndUtility(Index agI, Index t1, Index t2);
        /**\brief constructs the new, clustered, set of types for agI.
         *
         * newTypeList is the new, empty TypeClusterList for agI.
         * the function returns the number of (new, clustered) types for 
         * the agent.
         */
        size_t ConstructClusteredIndividualTypes(Index agI, 
            TypeClusterList* newTypeList);
        
    public:
        // Constructor, destructor and copy assignment.
        /**\brief Constructor used to construct a BayesianGameWithClusterInfo
         * from scratch.
         *
         * Given a past Dec-POMDP policy a new BG is constructed (not 
         * `bootstrapped' from a previous BG). This means that the resulting BG
         * look exactly the same as when constructing a regular
         * BayesianGameForDecPOMDPStage, only with the additional information
         * about state probabilies (the joint beliefs) and types.
         * Note that the type structure will contain for each agent a list with
         * exactly 1 type for each history
         */
        BayesianGameWithClusterInfo(
                const PlanningUnitDecPOMDPDiscrete* pu,
                const QFunctionJAOHInterface* q,
                const boost::shared_ptr<const PartialJointPolicyDiscretePure> &pastJPol,
                BGClusterAlgorithm clusterAlg=Lossless
        );
        /**\brief Constructor used to construct am empty BayesianGameWithClusterInfo
         */
        BayesianGameWithClusterInfo(
                const PlanningUnitDecPOMDPDiscrete* pu
        );
    protected:
        /**\brief This constructor builds a BayesianGameWithClusterInfo from a 
         * BGWCI for the previous stage (i.e., using `bootstrapping').
         *
         * Because we first need to compute the number of new types for each
         * agent by extending the type-lists from prevBG, it is impracticle to
         * call this constructor directly. Rather this constructor is called 
         * by ConstructExtendedBGWCI.
         */
        BayesianGameWithClusterInfo(
                const PlanningUnitDecPOMDPDiscrete* pu,
                const QFunctionJAOHInterface* q,
                Index t,
                const BGwCI_constPtr &prevBG,
                const boost::shared_ptr<const JointPolicyDiscretePure> &prevJPolBG,
                size_t nrAgents,
                const std::vector<size_t>& nrActions,
                const std::vector<size_t>& nrTypes,
                BGClusterAlgorithm clusterAlg
        );
        ///Test equivalence of agent agI's types t1 and t2
        bool TestExactEquivalence(Index agI, Index t1, Index t2) const;
 
        ///called by ConstructExtendedBGWCI
        void Extend();
        
        



    public:
        /// Copy constructor.
        BayesianGameWithClusterInfo(const BayesianGameWithClusterInfo& a);
        /// Destructor.
        virtual ~BayesianGameWithClusterInfo();
        /// Copy assignment operator
        BayesianGameWithClusterInfo& operator= (const BayesianGameWithClusterInfo& o);

        //operators:

        //data manipulation (set) functions:
        /**\brief Returns a losslessly clustered copy of this BG
         */
        BGwCI_sharedPtr Cluster();

        //get (data) functions:
        ///Return pointer to agent agI's tcI-th TypeCluster
        const TypeCluster* GetTypeCluster(Index agI, Index tcI) const
        {return _m_typeLists.at(agI)->at(tcI);};
        //
        //others:
        static BGwCI_sharedPtr ConstructExtendedBGWCI(
                const BGwCI_constPtr &pBG,
                const JointPolicyDiscretePure& prevJPolBG,
                //const JointPolicyPureVector& prevJPolBG,
                const QFunctionJAOHInterface* q
            );

        BGClusterAlgorithm GetClusterAlgorithm() const { return(_m_clusterAlgorithm); }

        double ComputeMarginalTypeProbability(Index agI, Index typeI) const;
        
        Index FindTypeClusterIndex(Index agI, const TypeCluster* tc, 
                Index aI, Index oI) const;

        boost::shared_ptr<const JointPolicyDiscretePure> GetPastJointPolicyPVFCBG() const
        { return _m_pBGJPol;};

        /** Prints a description of this  entire BayesianGameIdenticalPayoff 
         * to a string.*/
        std::string SoftPrint() const; 
        /**Print this BayesianGameIdenticalPayoff to cout.*/
        void Print() const
        { std::cout << SoftPrint();}
};


#endif /* !_BAYESIANGAMEWITHCLUSTERINFO_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
