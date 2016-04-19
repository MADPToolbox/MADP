/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _NULLPLANNERTOI_H_
#define _NULLPLANNERTOI_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "PlanningUnitTOIDecPOMDPDiscrete.h"

/** \brief NullPlannerTOI represents a planner which does nothing, but
 * can be used to instantiate a PlanningUnitTOIDecPOMDPDiscrete. */
class NullPlannerTOI : public PlanningUnitTOIDecPOMDPDiscrete
{
private:    
    
protected:
    
public:

    /// (default) Constructor
    NullPlannerTOI(TOIDecPOMDPDiscrete* p);

    NullPlannerTOI(size_t horizon, TOIDecPOMDPDiscrete* p);

    NullPlannerTOI(const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon, TOIDecPOMDPDiscrete* p);

    ~NullPlannerTOI(){};

    /// Only present to satisfy the interface.
    void Plan();
    /// Only present to satisfy the interface.
    double GetExpectedReward() const;
    /// Only present to satisfy the interface.
    boost::shared_ptr<JointPolicy> GetJointPolicy();

};


#endif /* !_NULLPLANNERTOI_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
