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
