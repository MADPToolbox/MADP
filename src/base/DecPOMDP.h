/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef _DECPOMDP_H_
#define _DECPOMDP_H_ 1

/* the include directives */

#include <iostream>
#include "Globals.h"
#include "DecPOMDPInterface.h"


/**\brief DecPOMDP is a simple implementation of DecPOMDPInterface.
 *
 *  It defines a couple of functions that relate to the (types of)
 *  rewards and discount factor.
 *
 * Conceptually an MultiAgentDecisionProcess that implements this interface, is
 * a Dec-POMDP: the system is cooperative and there is only 1 reward function.
  */
class DecPOMDP : 
    virtual public DecPOMDPInterface
{
    private:

        /// The discount parameter.
        /** When agents have different interests (the POSG setting),
         * they may also have different discount factors. For a
         * Dec-POMDP, however, we have one global discount factor
         * (which typically is 1.0 in the finite horizon case).*/
        double _m_discount;
        ///  Do the agents get rewards or costs?
        reward_t _m_rewardType;
    protected:

    public:

        // constructors etc.
        /// Default constructor. sets RewardType to REWARD and discount to 1.0.
        DecPOMDP();

        /// Sets the discount parameter to d.
        void SetDiscount(double d);
        /// Returns the discount parameter.
        double GetDiscount() const {return _m_discount;}
        /// Sets the reward type to reward_t r.
        /** At the moment only REWARD is supported. */
        void SetRewardType(reward_t r);
        /// Returns the reward type.
        reward_t GetRewardType() const {return _m_rewardType;}
        
        /// SoftPrints some information on the DecPOMDP.        
        std::string SoftPrint() const;

    ///Functions needed for POSGInterface:
        void SetDiscountForAgent(Index agentI, double d)
        {SetDiscount(d);}

        /// Returns the discount parameter.
        double GetDiscountForAgent(Index agentI) const
        {return GetDiscount();}

        /// Sets the reward type to reward_t r.
        void SetRewardTypeForAgent(Index agentI, reward_t r) 
        {SetRewardType(r);}

        /// Returns the reward type.
        reward_t GetRewardTypeForAgent(Index agentI) const 
        {return GetRewardType();}


};

#endif //! _DECPOMDP_H_

// Local Variables: ***
// mode:c++ ***
// End: ***
