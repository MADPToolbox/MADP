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
#ifndef _GENERALIZEDMAASTARPLANNER_H_
#define _GENERALIZEDMAASTARPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include <time.h>
#include <sys/times.h>
#include <fstream>
#include "boost/shared_ptr.hpp"
#include <float.h>
#include <limits.h>
#include "Globals.h"
#include "TimedAlgorithm.h"
#include "PartialPolicyPoolItemInterface.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
//use forward declarations when possible to speed up compilation:
//class PlanningUnitDecPOMDPDiscrete;
//class BayesianGameIdenticalPayoff;

//class JPolValPair;
//class QFunctionJAOHInterface;
//class TimedAlgorithm;
//class PolicyPoolJPolValPair;

class PartialPolicyPoolInterface;
//class PartialPolicyPoolItemInterface;

class JointPolicy;
class JointPolicyDiscrete;
class JointPolicyDiscretePure;
class JointPolicyPureVector;
class PartialJointPolicyDiscretePure;
class PartialJointPolicyPureVector;
class BayesianGameForDecPOMDPStage;
//template<class JP> class BayesianGameIdenticalPayoffSolver_T;

class Interface_ProblemToPolicyDiscretePure;
// keep these in header file so derived classes can use them
#define DEBUG_GMAA3 0
#define DEBUG_GMAA4 0
#define DEBUG_GMAA5 0

#define GMAA_SET_CBG_BOUNDS 1

/**\brief GeneralizedMAAStarPlanner is a class that represents the Generalized
 * MAA* planner class.
 *
 * This implements GMAA pretty much as described in #refGMAA (see
 * DOC-References.h).  The 'NEXT'  as described in #refGMAA, is called
 * 'ConstructAndValuateNextPolicies'.
 *
 * Additionally there is a function 'SelectPoliciesToProcessFurther'. (not to
 * be confused with the 'SELECT' function from #refGMAA !!!) Given the result
 * of ConstructAndValuateNextPolicies, SelectPoliciesToProcessFurther
 * determines which of these will actually be added to the policy pool. I.e.,
 * ConstructAndValuateNextPolicies and SelectPoliciesToProcessFurther together
 * form 'NEXT'.
 *
 * The 'SELECT' function as described in #refGMAA is implemented by the policy
 * pool (see PartialPolicyPoolInterface) itself.
 *
 *
 * \sa GMAA_MAAstar, GMAA, PartialPolicyPoolInterface
 *
 * */
class GeneralizedMAAStarPlanner :
    public TimedAlgorithm
{

    private:

        size_t _m_deadline;

        ///the best found policy
        boost::shared_ptr<JointPolicyDiscretePure> _m_foundPolicy;
        ///the expected reward of the best found policy
        double _m_expectedRewardFoundPolicy;

        /// The highest lowerbound found so far.
        double _m_maxLowerBound;

        /**a counter that maintains the maximum size of the policy pool during 
         * the planning process.*/
        LIndex _m_maxJPolPoolSize;

        /// Initialize the planner.
        void Initialize();

    protected:

        ///the level of verboseness, default=0, >0 verbose, <0 silent
        int _m_verboseness;

        /**Pointer to an file stream to store the intermediate (timing) results.
         * */
        std::ofstream* _m_intermediateResultFile;

        bool _m_useSparseBeliefs;

        bool _m_saveIntermediateTiming;
        std::string _m_intermediateTimingFilename;

        size_t _m_bgCounter;
        std::string _m_bgBaseFilename;

        /*the # of joint policies for Bayesian Games that are evaluated.*/
        LIndex _m_nrJPolBGsEvaluated;
    
        size_t _m_nrPoliciesToProcess;

        /**when the heuristic is not admissible, or the past reward is an approximation,
         * we may add some slack such that good policies are not pruned
         */
        double _m_slack;


        //keep counts
        /**_m_expanded_childs[t] contains the number of child nodes that were 
         * expanded 'at stage t'. That means
         *
         * selected parent of depth t = \varphi^t
         * -> child has depth t+1 = (varphi^t, \delta^t)
         * (so the child nodes are depth t+1 !)
         */
        std::vector<size_t> _m_expanded_childs;

        //because BFS does not expand all child nodes anymore, we need a different way of figuring out
        //how many nodes it would have expanded: we maintain the max number by counting the number
        //of joint policies for all encountered BGs 
        //(only implemented in GMAA_MAAstarCluster!)
        std::vector<LIndex> _m_max_expanded_childs;

        // here derived planners can store pointers to BGs they
        // instantiate, so they can be cleaned up properly when
        // resetting or deleting the planner
        std::vector<BayesianGameForDecPOMDPStage*> _m_pointersToAllBGTS;


        ///every derived class must implement this function as follows:
        /**GetThisFromMostDerivedPU()
         * { return this; }
         * Giving us access to the Interface_ProblemToPolicyDiscretePure
         * in this base class.
         */
        virtual Interface_ProblemToPolicyDiscretePure* 
            GetThisFromMostDerivedPU() = 0;

        // The 'NEXT' and 'SELECT' functions
    
        /**\brief The 'NEXT' function as described in #refGMAA.
         *
         * The function that from a given <jpol,val> pair construct a new
         * (ordered by value->priority_queue) set of joint policies.
         * This function should be overriden in derived classes to get 
         * different planning behavior.*/
        virtual bool ConstructAndValuateNextPolicies(
                const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi,
                const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
                bool &cleanUpPPI) = 0;

        /**\brief Limits the policies to be further examined. 
         *
         * Of the <jpol,val> pairs found by ConstructAndValuateNextPolicies,
         * we may not want to process all of them further. This function 
         * performs a selection.
         * This function should be overriden in derived classes to get 
         * different planning behavior.*/
        virtual void SelectPoliciesToProcessFurther(
                const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
                bool are_LBs, double bestLB)=0;

        /**\brief return a new policy pool.
         *
         * this function must be implemented by a derived class and return a
         * pointer to a PartialPolicyPoolInterface object. This way,the derived class
         * can determine the implementation of the policy pools used and thus
         * the 'SELECT' functions as described in #refGMAA.
         *
         * it returns a pointer to a PartialPolicyPoolInterface that is created by
         * 'new' therefore, do not forget to 'delete'!!
         */
        virtual boost::shared_ptr<PartialPolicyPoolInterface> NewPP() const=0;
        /**\brief return a new policy pool item. 
         *
         * This function must be implemented by a derived class and return a
         * pointer to a newly created PolicyPoolItem object. This way,the
         * derived class can determine the implementation of the policy pools
         * used and thus the 'SELECT' functions as described in #refGMAA.
         *
         * it returns a pointer to a PartialPolicyPoolInterface that is created by
         * 'new' therefore, do not forget to 'delete'!!
         */
//        virtual PartialPolicyPoolItemInterface* NewPPI()=0;
        /**\brief Overloaded form of NewPP().
         *
         * Creates a PartialPolicyPoolItemInterface which contains joint policy p,
         * with value v.*/
        virtual boost::shared_ptr<PartialPolicyPoolItemInterface> NewPPI(
                const boost::shared_ptr<PartialJointPolicyDiscretePure> &p,
                double v) const=0;

        /**\brief return a new policy.
         *
         * Different versions of GMAA may make use of different implementations
         * of policies.
         * This function must be implemented by a derived class and return a
         * pointer to a newly created PolicyPoolItem object. This way,the
         * derived class can determine the implementation of the policy. 
         *
         * it returns a pointer to a PartialJointPolicyDiscretePure that is 
         * created by
         * 'new' therefore, do not forget to 'delete'!!
         */
        virtual boost::shared_ptr<PartialJointPolicyDiscretePure> NewJPol() const=0;


        /**\brief Returns the k best-ranked (partial) joint policies.
         *
         * An implementation of the 'SELECT' function, that returns the 
         * k (partial) joint policies with the highest heuristic values. */
        void SelectKBestPoliciesToProcessFurther(
            const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies,
            bool are_LBs, double bestLB, size_t k);


        //auxiliary functions 
        //\todo NOTE: all these functions are now performed in BayesianGameForDecPOMDPStage, this class should use that class.

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
         * firstOHtsI - a vector that specifies the index of the first 
         *              observation history in time step ts for each agent 
         *              (this functions
         *              as the offset in the conversion BG->DecPOMDP index 
         *              conversion).
         *
         * returns a new JointPolicyPureVector (so it must be explicitly 
         * deleted)
         * */
        virtual boost::shared_ptr<PartialJointPolicyDiscretePure> ConstructExtendedJointPolicy(
                const PartialJointPolicyDiscretePure& jpolPrevTs
                , const JointPolicyDiscretePure& jpolBG
                , const std::vector<size_t>& nrOHts
                , const std::vector<Index>& firstOHtsI) = 0;

        /// This should reset the planner, so it can be started from the beginning.
        virtual void ResetPlanner() = 0;

        void Prune(PartialPolicyPoolInterface& JPVs, size_t k);

        //template<class JP>
        void SetCBGbounds(const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi,
                          const boost::shared_ptr<BayesianGameIdenticalPayoffSolver> &bgips,
                          bool is_last_ts,
                          double discount)
        {
            size_t ts = ppi->GetJPol()->GetDepth();
            double pastReward_prevTs = ppi->GetJPol()->GetPastReward();
#if GMAA_SET_CBG_BOUNDS
            double CBGlowerbound=GetMaxLowerBound() - pastReward_prevTs;
            double CBGupperbound=DBL_MAX;
            if(is_last_ts && discount==1.0)
                CBGupperbound= ppi->GetValue() - pastReward_prevTs;
            bgips->SetCBGlowerBound(CBGlowerbound);
            bgips->SetCBGupperBound(CBGupperbound);
            
            if(_m_verboseness >= 1)
                std::cout << "GMAA ts " << ts << " nrJT "
                          << bgips->GetBGIPI()->GetNrJointTypes() << " CBG bounds [ "
                          << CBGlowerbound << " , " << CBGupperbound << " ] "
                          << "GMAA lb " << GetMaxLowerBound()
                          << " pastR " << pastReward_prevTs << std::endl;
#endif
            //in the worst case this is the number of children that will be expanded:
            if(_m_max_expanded_childs.size()<=ts)
                _m_max_expanded_childs.resize(ts+1,0);

            try {
                _m_max_expanded_childs.at(ts) += bgips->GetBGIPI()->GetNrJointPolicies();
            }
            catch(EOverflow& e){ 
                std::cout << "Warning, joint policy indices are overflowing, max expanded children will be set to 0" << std::endl;
                _m_max_expanded_childs.at(ts) = 0;
            }

        }
        
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        GeneralizedMAAStarPlanner(
            int verbose_level=0,
            double slack = 0.0
            );


        /// Destructor.
        ~GeneralizedMAAStarPlanner();
        /// Copy assignment operator
        GeneralizedMAAStarPlanner& operator=(const 
                GeneralizedMAAStarPlanner& o);
        //operators:

        //data manipulation (set) functions:

        void SetIntermediateResultFile(std::ofstream& of)
            {_m_intermediateResultFile = &of;}
        void SetIntermediateTimingFilename(const std::string &filename);
        void SetSaveAllBGs(const std::string &filename)
            { _m_bgBaseFilename=filename; }
        void SetVerbose(int verbose);

        void Plan();
        
        //get (data) functions:
        //do not define these functions here (that requires inclusion of 
        //header files...)
        boost::shared_ptr<JointPolicy> GetJointPolicy();
        boost::shared_ptr<JointPolicyDiscrete> GetJointPolicyDiscrete();
        boost::shared_ptr<JointPolicyDiscretePure> GetJointPolicyDiscretePure();

        double GetExpectedReward() const
            { return(_m_expectedRewardFoundPolicy); }
        LIndex GetMaxJPolPoolSize() const
            { return(_m_maxJPolPoolSize);}
        LIndex GetNrEvaluatedJPolBGs() const
        {return  _m_nrJPolBGsEvaluated;}
        double GetMaxLowerBound() const { return(_m_maxLowerBound); }

        void SetDeadline(size_t deadlineInS);
};


#endif /* !_GENERALIZEDMAASTARPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
