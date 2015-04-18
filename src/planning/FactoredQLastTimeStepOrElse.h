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
#ifndef _FACTOREDQLASTTIMESTEPORELSE_H_
#define _FACTOREDQLASTTIMESTEPORELSE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "FactoredQFunctionStateJAOHInterface.h"
#include "QFunctionForFactoredDecPOMDP.h"

class PlanningUnitFactoredDecPOMDPDiscrete;

/**\brief FactoredQLastTimeStepOrElse is a class that represents a Q-function that
 * is factored at the last stage, and non factored for earlier stages.
 *
 * This is the type of Q-function used by #refGMAAELSI.
 * */
class FactoredQLastTimeStepOrElse : 
    public FactoredQFunctionStateJAOHInterface,
    public QFunctionForFactoredDecPOMDP
{
private:    

    bool _m_initialized;
    
    ///The number of local reward functions
    /**(this is equal the the nr. of local Q-functions, as we only represent
     * a factored Q-function for the last stage.)
     */
    size_t _m_nrLRFs;
    std::vector<Scope> _m_agentScopes;


protected:
    
public:

    /// (default) Constructor
    FactoredQLastTimeStepOrElse(const PlanningUnitFactoredDecPOMDPDiscrete *puf);
    FactoredQLastTimeStepOrElse(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf);
    
    /// Destructor.
    virtual ~FactoredQLastTimeStepOrElse();

    size_t GetNrLQFs(Index stage) const;
    const Scope& GetAgentScopeForLQF(Index e, Index stage) const;
    const Scope& GetStateFactorScopeForLQF(Index lqf, Index stage) const;

    void Initialize();
    void DeInitialize();
    void Compute();


    
    //implement FactoredQFunctionStateJAOHInterface:

    double GetLocalQValue(
            Index LRF, 
            Index stage,
            const std::vector<Index>& sfacValues,  
            const std::vector<Index>& aoHistIs, 
            Index agSc_jaI //joint group action index
    ) const;
    
    /** Get the Q for LRF, for local state, local aoh and local joint action
     * only then with only 'joint' (i.e., group) indices
     */
    double GetLocalQValue(Index LRF, Index stage, Index sfSC_sI,  
            Index agSC_aoHistI, Index agSc_jaI) const    
    { throw E("Not yet implemented:  GetLocalQValue(Index LRF, Index sfSC_sI, Index agSC_aoHistI, Index agSc_jaI) "); };

    double GetLocalQValue(Index LRF, Index aoHistI, Index jaI) const
    { throw E("FactoredQLastTimeStepOrElse::GetLocalQValue(Index LRF, Index aoHistI, Index jaI) is not implemented"); };


};


#endif /* !_FACTOREDQLASTTIMESTEPORELSE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
