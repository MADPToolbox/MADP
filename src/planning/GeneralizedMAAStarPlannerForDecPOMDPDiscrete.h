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
#ifndef _GENERALIZEDMAASTARPLANNERFORDECPOMDPDISCRETE_H_
#define _GENERALIZEDMAASTARPLANNERFORDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <time.h>
#include <sys/times.h>
#include <fstream>

//#include "Globals.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "GeneralizedMAAStarPlanner.h"

//needed becaise of inline at bottom of file
#include "QFunctionJAOHInterface.h"

/**\brief GeneralizedMAAStarPlannerForDecPOMDPDiscrete is a class that 
 * represents the Generalized MAA* planner.
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
 * This class also implements a 'NEXT' and 'SELECT' functions that can be used
 * by derived classes.
 *
 *
 * \sa GMAA_MAAstar, GMAA, PartialPolicyPoolInterface
 *
 * */
class GeneralizedMAAStarPlannerForDecPOMDPDiscrete :
    public PlanningUnitDecPOMDPDiscrete, //don't make virtual (everything will fail!
    public GeneralizedMAAStarPlanner
{
    private:    
    protected:
        ///A pointer to the heuristic used by this 
        //GeneralizedMAAStarPlannerForDecPOMDPDiscrete
        QFunctionJAOHInterface* _m_qHeuristic;

//Functions that can be overriden by derived classes to influence the working of
//GMAA:
        
        /// Returns a new policy of the type used by the GMAA implementation
        virtual boost::shared_ptr<PartialJointPolicyDiscretePure> NewJPol() const;
        ///Return a new PartialPolicyPoolItemInterface*.
        /**This function returns a pointer to new instance of the 
         * PartialPolicyPoolItemInterface used by this class.
         */
        virtual boost::shared_ptr<PartialPolicyPoolItemInterface> NewPPI(
            const boost::shared_ptr<PartialJointPolicyDiscretePure> &jp,
            double v) const;
        ///Return a new PartialPolicyPoolInterface*.
        /**This function returns a pointer to new instance of the 
         * PartialPolicyPoolInterface used by this class.
         */
        virtual boost::shared_ptr<PartialPolicyPoolInterface> NewPP() const;

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
                , const std::vector<Index>& firstOHtsI);


        double GetHeuristicQ(Index joahI, Index jaI) const;

        //using GeneralizedMAAStarPlanner::SetCBGbounds;
        /** Function to set the bounds of the CBG.
         *
         *  This is put here because GeneralizedMAAStarPlanner cannot call
         *  GetHorizon(). (that function is provided by PlanningUnitDecPOMDPDiscrete 
         *  from which this class inherits).
         */
        void SetCBGbounds(const boost::shared_ptr<PartialPolicyPoolItemInterface> &ppi,
                          const boost::shared_ptr<BayesianGameIdenticalPayoffSolver> &bgips)
        {
            PJPDP_sharedPtr jpolPrevTs = ppi->GetJPol();//jpol^ts-1
            size_t ts = jpolPrevTs->GetDepth();     // = depth = ts(jpolPrevTs) + 1
            bool is_last_ts = (ts ==  GetHorizon() - 1);
            this->GeneralizedMAAStarPlanner::SetCBGbounds(ppi,bgips,is_last_ts,GetDiscount());
        }

    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        GeneralizedMAAStarPlannerForDecPOMDPDiscrete(
            const PlanningUnitMADPDiscreteParameters &params,
            size_t horizon=3, 
            DecPOMDPDiscreteInterface* p=0,
            int verbose_level=0
            );

        GeneralizedMAAStarPlannerForDecPOMDPDiscrete(
            size_t horizon=3, 
            DecPOMDPDiscreteInterface* p=0);

        /// Destructor.
        //~GeneralizedMAAStarPlannerForDecPOMDPDiscrete();
        /// Copy assignment operator
        GeneralizedMAAStarPlannerForDecPOMDPDiscrete& operator=(const 
                GeneralizedMAAStarPlannerForDecPOMDPDiscrete& o);
        
        void SetQHeuristic(QFunctionJAOHInterface& q)
            {_m_qHeuristic = &q;}
        void SetQHeuristic(QFunctionJAOHInterface* q)
            {_m_qHeuristic = q;}

        virtual GeneralizedMAAStarPlannerForDecPOMDPDiscrete* 
            GetThisFromMostDerivedPU()
        { return this; }
        //some functions necessary because we derive from 
        //PlanningUnitDecPOMDPDiscrete
        void Plan() 
        {   GeneralizedMAAStarPlanner::Plan();  }
        double GetExpectedReward() const 
        {   return GeneralizedMAAStarPlanner::GetExpectedReward();  }
        boost::shared_ptr<JointPolicy> GetJointPolicy()
        {   return GeneralizedMAAStarPlanner::GetJointPolicy();    }
        
};

inline
double GeneralizedMAAStarPlannerForDecPOMDPDiscrete::
GetHeuristicQ(Index joahI, Index jaI) const
{return(_m_qHeuristic->GetQ(joahI, jaI));}



#endif /* !_GENERALIZEDMAASTARPLANNERFORDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
