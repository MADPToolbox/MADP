/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _NULLPLANNERFACTORED_H_
#define _NULLPLANNERFACTORED_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "PlanningUnitFactoredDecPOMDPDiscrete.h"

/** \brief NullPlannerFactored represents a planner which does nothing, but
 * can be used to instantiate a PlanningUnitDecFactoredPOMDPDiscrete. */
class NullPlannerFactored : 
    public PlanningUnitFactoredDecPOMDPDiscrete
{
private:    
    
protected:
    
public:

    /// (default) Constructor
    NullPlannerFactored(FactoredDecPOMDPDiscreteInterface* p);

    NullPlannerFactored(size_t horizon, FactoredDecPOMDPDiscreteInterface* p);

    NullPlannerFactored(const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon, FactoredDecPOMDPDiscreteInterface* p);

    ~NullPlannerFactored(){};

    /// Only present to satisfy the interface.
    void Plan();
    /// Only present to satisfy the interface.
    double GetExpectedReward() const;
    /// Only present to satisfy the interface.
    boost::shared_ptr<JointPolicy> GetJointPolicy();

};


#endif /* !_NULLPLANNERFACTORED_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
