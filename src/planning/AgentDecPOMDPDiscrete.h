/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENTDECPOMDPDISCRETE_H_
#define _AGENTDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "SimulationAgent.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief AgentDecPOMDPDiscrete represents an agent in a discrete
 * DecPOMDP setting. */
class AgentDecPOMDPDiscrete : public SimulationAgent
{
private:    

    const PlanningUnitDecPOMDPDiscrete *_m_pu;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentDecPOMDPDiscrete(const PlanningUnitDecPOMDPDiscrete *pu, Index id) :
        SimulationAgent(id), _m_pu(pu) {};

    /// Copy constructor.
    AgentDecPOMDPDiscrete(const AgentDecPOMDPDiscrete& a) :
        SimulationAgent(a),
        _m_pu(a._m_pu){};

    const PlanningUnitDecPOMDPDiscrete* GetPU() const
        { return(_m_pu); }

};


#endif /* !_AGENTDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
