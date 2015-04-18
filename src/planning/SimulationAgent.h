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
#ifndef _SIMULATIONAGENT_H_
#define _SIMULATIONAGENT_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/// SimulationAgent represents an agent in for class Simulation.
class SimulationAgent 
{
private:    
    
    /// The index of this SimulationAgent, should be unique.
    Index _m_id;

    /// Whether we want the agent to be verbose.
    bool _m_verbose;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    SimulationAgent(Index id, bool verbose=false) :
        _m_id(id), _m_verbose(verbose){};

    /// Destructor.
    virtual ~SimulationAgent(){};

    /// Retrieves the index of this agent.
    virtual Index GetIndex() const { return(_m_id); }

    /// Sets the index of this agent.
    virtual void SetIndex(Index id) { _m_id=id; }

    /// Set whether this agent should be verbose.
    virtual void SetVerbose(bool verbose) { _m_verbose=verbose; }
    
    /// If true, the agent will report more.
    virtual bool GetVerbose() const { return(_m_verbose); }

    /// Will be called before an episode, to reinitialize the agent.
    virtual void ResetEpisode() = 0;

    /// Return some information about this agent.
    virtual std::string SoftPrint() const
    {
        std::stringstream ss;
        ss << "SimulationAgent id " << GetIndex();
        return(ss.str());
    }

    /// Print out some information about this agent.
    void Print() const {std::cout << SoftPrint(); }

};


#endif /* !_SIMULATIONAGENT_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
