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
#ifndef _JESPEXHAUSTIVEPLANNER_H_
#define _JESPEXHAUSTIVEPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "JointPolicyPureVector.h"
#include "JointPolicy.h"

/// JESPExhaustivePlanner plans with the Exhaustive JESP algorithm.
/** It has been presented in refJESP (see DOC-References.h).
 */
class JESPExhaustivePlanner : public PlanningUnitDecPOMDPDiscrete
{
    private:    
        //the best found policy
        boost::shared_ptr<JointPolicyPureVector> _m_foundPolicy;
        //the expected reward of the best found policy
        double _m_expectedRewardFoundPolicy;
        //intermediate result from ExhaustiveBestResponse
        //JointPolicyPureVector _m_exhBRBestPol;
    protected:
    
    public:
        
        // Constructor, destructor and copy assignment.
        // (default) Constructor
        //JESPExhaustivePlanner();
        JESPExhaustivePlanner(
            const PlanningUnitMADPDiscreteParameters &params,
            size_t horizon, DecPOMDPDiscreteInterface* p);
        JESPExhaustivePlanner(int horizon, DecPOMDPDiscreteInterface* p);

        //operators:

        //data manipulation (set) functions:
        /**The methods that performs the planning according to the Exhaustive
         * JESP algorithm. */
        void Plan();
        double ExhaustiveBestResponse(JointPolicyPureVector* jpol, int agentI);

        //get (data) functions:
        boost::shared_ptr<JointPolicy> GetJointPolicy()
            { return(_m_foundPolicy); }
        boost::shared_ptr<JointPolicyDiscrete> GetJointPolicyDiscrete()
            { return(_m_foundPolicy); }
        boost::shared_ptr<JointPolicyPureVector> GetJointPolicyPureVector()
            { return(_m_foundPolicy); }
        double GetExpectedReward(void) const
            { return(_m_expectedRewardFoundPolicy); }

};


#endif /* !_JESPEXHAUSTIVEPLANNER_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
