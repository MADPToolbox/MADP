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
