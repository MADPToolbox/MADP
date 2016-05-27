/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
#ifndef _AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_
#define _AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentTOIFullyObservableSynced.h"
#include <vector>

/**AgentTOIFullyObservableSyncedSpecialReward represents an
 * AgentTOIFullyObservableSynced. After acting, each agent's
 * GetSpecialReward() method gets called, and those reward are
 * incorporated in the overall reward. */
class AgentTOIFullyObservableSyncedSpecialReward : 
    public AgentTOIFullyObservableSynced
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentTOIFullyObservableSyncedSpecialReward(
        const PlanningUnitDecPOMDPDiscrete *pu,
        Index id) :
        AgentTOIFullyObservableSynced(pu,id){};

    /// Copy constructor.
    AgentTOIFullyObservableSyncedSpecialReward(
        const AgentTOIFullyObservableSyncedSpecialReward& a) :
        AgentTOIFullyObservableSynced(a){};

    /// Destructor.
    ~AgentTOIFullyObservableSyncedSpecialReward(){};
   
    virtual double GetSpecialReward() = 0;

};


#endif /* !_AGENTTOIFULLYOBSERVABLESYNCEDSPECIALREWARD_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
