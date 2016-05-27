/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _NULLPLANNER_H_
#define _NULLPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "PlanningUnitDecPOMDPDiscrete.h"

/** \brief NullPlanner represents a planner which does nothing, but
 * can be used to instantiate a PlanningUnitDecPOMDPDiscrete. */
class NullPlanner : public PlanningUnitDecPOMDPDiscrete
{
private:    
    
protected:
    
public:

    /// (default) Constructor
    NullPlanner(DecPOMDPDiscreteInterface* p);

    NullPlanner(size_t horizon, DecPOMDPDiscreteInterface* p);

    NullPlanner(const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon, DecPOMDPDiscreteInterface* p);

    ~NullPlanner(){};

    /// Only present to satisfy the interface.
    void Plan();
    /// Only present to satisfy the interface.
    double GetExpectedReward() const;
    /// Only present to satisfy the interface.
    boost::shared_ptr<JointPolicy> GetJointPolicy();

};


#endif /* !_NULLPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
