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
#ifndef _AGENTQMDP_H_
#define _AGENTQMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AgentSharedObservations.h"
#include "QFunctionJAOHInterface.h"
#include "JointBeliefSparse.h"
#include "QTable.h"



/**AgentQMDP represents an agent which uses a QMDP-based policy. */
class AgentQMDP : public AgentSharedObservations
{
private:    
    
    QTable _m_Q;

    size_t _m_t;

    JointBeliefSparse _m_jb;
    
    Index _m_prevJaI;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentQMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
              const QTable &Q);

    /// Copy constructor.
    AgentQMDP(const AgentQMDP& a);

    /// Destructor.
    ~AgentQMDP();

    Index Act(Index joI);

    void ResetEpisode();

};


#endif /* !_AGENTQMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
