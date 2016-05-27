/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BAYESIANGAMEFORDECPOMDPSTAGEINTERFACE_H_
#define _BAYESIANGAMEFORDECPOMDPSTAGEINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "boost/shared_ptr.hpp"

class PartialJointPolicyDiscretePure;
class JointPolicyDiscretePure;
class FactoredDecPOMDPDiscreteInterface;
class PlanningUnitFactoredDecPOMDPDiscrete;

/** \brief BayesianGameForDecPOMDPStageInterface is a class that represents 
 * the base class for all Bayesian games that are used to represent a 
 * stage of a Dec-POMDP (e.g., in GMAA*).
 * */
class BayesianGameForDecPOMDPStageInterface 
{
    private:    
    
        ///The stage (time step) that this BG represents
        Index _m_t;
        ///Stores pointer to the past policy - perhaps not needed?
        boost::shared_ptr<const PartialJointPolicyDiscretePure> _m_pJPol;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// Constructor without arguments, needed for serialization.
        BayesianGameForDecPOMDPStageInterface();

        BayesianGameForDecPOMDPStageInterface(const 
                boost::shared_ptr<const PartialJointPolicyDiscretePure> &pastJPol);
        BayesianGameForDecPOMDPStageInterface(Index t);
        /// Destructor.
        virtual ~BayesianGameForDecPOMDPStageInterface(){};

        Index GetStage() const
        {return _m_t;}
        boost::shared_ptr<const PartialJointPolicyDiscretePure> GetPastJointPolicy() const
        { return _m_pJPol;}
        
        /**Returns the (expected) immediate reward for jtI, jaI
         */
        virtual double GetImmediateReward(Index jtI, Index jaI,
                                          const FactoredDecPOMDPDiscreteInterface *fd) const=0;
        /**When performing a lot of GetImmediateReward calls we can first 
         * compute a cache of immediate rewards, to speed things up.
         */
        virtual void ComputeAllImmediateRewards(const FactoredDecPOMDPDiscreteInterface *fd) = 0;
        /**We can also clear this cache.
         */
        virtual void ClearAllImmediateRewards() = 0;
        /**\brief Compute the discounted expected imm reward for jpolBG.
         */
        virtual double ComputeDiscountedImmediateRewardForJPol(
                const boost::shared_ptr<JointPolicyDiscretePure> &jpolBG,
                const PlanningUnitFactoredDecPOMDPDiscrete *pu) const = 0;
};


#endif /* !_BAYESIANGAMEFORDECPOMDPSTAGEINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
