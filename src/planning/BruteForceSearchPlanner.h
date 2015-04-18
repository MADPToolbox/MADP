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
#ifndef _BRUTEFORCESEARCHPLANNER_H_
#define _BRUTEFORCESEARCHPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "JointPolicyPureVector.h"

/// BruteForceSearchPlanner implements an exact solution algorithm.
/** Uses brute force search, i.e., it loops over all joint policies,
 * computes the the value of each of them, and stores the one with the
 * highest value. This results necessarily in the optimal value. */
class BruteForceSearchPlanner : public PlanningUnitDecPOMDPDiscrete
{
    private:    
    boost::shared_ptr<JointPolicyPureVector> _m_foundPolicy;
    double _m_expectedRewardFoundPolicy;
    
    protected:

    public:
    
    // Constructor, destructor and copy assignment.
    // (default) Constructor
    BruteForceSearchPlanner(size_t horizon=3, DecPOMDPDiscreteInterface* p=0);

    BruteForceSearchPlanner(const PlanningUnitMADPDiscreteParameters &params,
                            size_t horizon=3, DecPOMDPDiscreteInterface* p=0);

    //operators:

    //data manipulation (set) functions:
    ///The methods that performs the planning.
    void Plan();
    
    //get (data) functions:
    boost::shared_ptr<JointPolicy> GetJointPolicy(void){ return(_m_foundPolicy); }
    boost::shared_ptr<JointPolicyDiscrete> GetJointPolicyDiscrete(void)
        { return(_m_foundPolicy); }
    boost::shared_ptr<JointPolicyPureVector> GetJointPolicyPureVector(void)
        { return(_m_foundPolicy); }

    double GetExpectedReward(void) const 
        { return(_m_expectedRewardFoundPolicy); }

};


#endif /* !_BRUTEFORCESEARCHPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
