/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
