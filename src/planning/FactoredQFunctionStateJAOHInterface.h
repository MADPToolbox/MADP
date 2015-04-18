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
#ifndef _FACTOREDQFUNCTIONINTERFACE_H_
#define _FACTOREDQFUNCTIONINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionJAOHInterface.h"
#include "Scope.h"

class FactoredPlanningUnitDecPOMDPDiscrete;

/**FactoredQFunctionStateJAOHInterface represents Q-value functions for factored
 * discrete Dec-POMDPs. */
class FactoredQFunctionStateJAOHInterface : public QFunctionJAOHInterface
{
private:    

protected:
    
public:

    /// Destructor
    virtual ~FactoredQFunctionStateJAOHInterface(){};

    ///Return the number of local Q value functions for \a stage
    virtual size_t GetNrLQFs(Index stage) const = 0;

    /**\brief Get local Q-value using individual indices.
     *
     * Get the local Q-value for 
     *  \li the e-th local Q-value function
     *  \li for stage (time-step) \a stage
     *  \li the vector of state factor value indices \a sfacValues,
     *      for the state factors in the scope of the e-th local Q-function
     *      (i.e. the state factors corresponding to 
     *      GetStateFactorScopeForLQF(e, stage) )
     *  \li the vector of individual action-observation history indices for
     *      the agents in the agent scope of the e-th local Q-function
     *  \li group action jaI_e. 
     *
     * jaI_e is the group index of the agents in GetAgentScopeForLQF( lqf, 
     * stage). Is is easily computed by
     *
     * IndexTools::IndividualToJointIndices(indices, nrActions)
     *
     * where the former argument is are the individual action indices of the 
     * agents in the scope, and the latter are the number of individual actions
     * they have.
     *
     */
    virtual double GetLocalQValue(
            Index e, 
            Index stage,
            const std::vector<Index>& sfacValues_e,  
            const std::vector<Index>& aoHistIs_e, 
            const std::vector<Index>& acs_e 
    ) const
    {
        throw E("FactoredQFunctionStateJAOHInterface function not overriden?!");
    }
    /**\brief Get local Q-value using individual and group indices.
     *
     * Get the local Q-value for 
     *  \li the e-th local Q-value function
     *  \li for stage (time-step) \a stage
     *  \li the vector of state factor value indices \a sfacValues,
     *      for the state factors in the scope of the e-th local Q-function
     *      (i.e. the state factors corresponding to 
     *      GetStateFactorScopeForLQF(e, stage) )
     *  \li the vector of individual action-observation history indices for
     *      the agents in the agent scope of the e-th local Q-function
     *  \li group action jaI_e. 
     *
     * jaI_e is the group index of the agents in GetAgentScopeForLQF( lqf, 
     * stage). Is is easily computed by
     *
     * IndexTools::IndividualToJointIndices(indices, nrActions)
     *
     * where the former argument is are the individual action indices of the 
     * agents in the scope, and the latter are the number of individual actions
     * they have.
     *
     */
    virtual double GetLocalQValue(
            Index e, 
            Index stage,
            const std::vector<Index>& sfacValues_e,  
            const std::vector<Index>& aoHistIs_e, 
            Index jaI_e //joint group action index
    ) const = 0;
    /**\brief Get local Q-value using individual and group indices.
     *
     * Get the local Q-value for 
     *  \li the e-th local Q-value function
     *  \li for stage (time-step) \a stage
     *  \li ('local') state sfSC_sI, 
     *  \li the vector of individual action-observation history indices for
     *      the agents in the agent scope of the e-th local Q-function
     *  \li group action jaI_e. 
     *
     * jaI_e is the group index of the agents in GetAgentScopeForLQF( lqf, 
     * stage). Is is easily computed by
     *
     * IndexTools::IndividualToJointIndices(indices, nrActions)
     *
     * where the former argument is are the individual action indices of the 
     * agents in the scope, and the latter are the number of individual actions
     * they have.
     *
     */
    virtual double GetLocalQValue(
            Index e, 
            Index stage,
            Index sfSC_sI,  
            const std::vector<Index>& aoHistIs_e, 
            Index jaI_e //joint group action index
    ) const 
    {
        throw E("FactoredQFunctionStateJAOHInterface::  double GetLocalQValue(\
            Index e, \
            Index stage,\
            Index sfSC_sI,\
            const std::vector<Index>& aoHistIs_e, \
            Index jaI_e) \
            should be overridden!");
    };
    /**\brief Get local Q-value using group indices.
     *
     * Get the local Q-value for 
     *  \li the LQF-th Q-value function
     *  \li for stage (time-step) \a stage
     *  \li ('local') state sfSC_sI, 
     *  \li the local aoHist: agSC_aoHistI
     *  \li local joint action agSC_jaI
     *
     * using only 'joint' (i.e., group) indices.
     *
     * I.e., 
     *  \li sfSC_sI is a group index for those state factors that are in
     * the scope GetStateFactorsForLQF(Index lqf, Index stage)
     *  \li agSC_aoHistI is a group index for the action-observation histories
     *  of the agents in GetAgentScopeForLQF(Index lqf, Index stage). See below
     *  for more details.
     *  \li agSc_jaI is the group index of those agent's actions. Is is easily 
     *  computed by IndexTools::IndividualToJointIndices(indices, nrActions)
     *
     * Computing the joint aoHist index agSC_aoHistI is somewhat more 
     * complicated than agSc_jaI, because not all indices are valid for the 
     * relevant stage. In particular 
     */
    virtual double GetLocalQValue(
            Index e, 
            Index stage, 
            Index sfSC_sI,  
            Index agSC_aoHistI, 
            Index agSC_jaI) const = 0;


    ///Return the State factor Scope for the \a lqf-th LQF at time-step \a stage
    virtual const Scope& GetStateFactorScopeForLQF(Index lqf, Index stage) const=0;
    ///Return the AgentScope for the \a lqf-th LQF at time-step \a stage
    virtual const Scope& GetAgentScopeForLQF(Index lqf, Index stage) const = 0;

};

#endif /* !_FACTOREDQFUNCTIONINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
