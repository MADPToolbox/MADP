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
#ifndef _PLANNINGUNIT_H_
#define _PLANNINGUNIT_H_ 1

/* the include directives */
#include <iostream>
#include <stdlib.h>
#include "Globals.h"
#include "MultiAgentDecisionProcessInterface.h"

#define DEBUG_PU_CONSTRUCTORS 0

class JointPolicy;

/** \brief PlanningUnit represents a planning unit, i.e., a planning
 * algorithm.
 *
 * This is a very general class, that only manages the planning
 * horizon and can initialize the random number generator (srand).
*/
class PlanningUnit
{
    private:

    /// Pointer to the problem.
    MultiAgentDecisionProcessInterface *_m_problem;

    /// Used by GetNextAgentIndex().
    Index _m_agentI;
    
    /// The planning horizon: MAXHORIZON for infinite horizon. */
    size_t _m_horizon;
    /// The random seed.
    int _m_seed;

    protected:

    public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor

    /// Constructor which sets the horizon and the problem.
    PlanningUnit(size_t horizon,
                 MultiAgentDecisionProcessInterface* p) :
        _m_problem(p),
        _m_agentI(0),
        _m_horizon(horizon),
        _m_seed(0)
        {}

    /// Destructor.
    virtual ~PlanningUnit(){}

    //operators:

    //data manipulation (set) functions:
    /// Stores the random seed and calls InitSeed().
    void SetSeed(int s){ _m_seed = s; InitSeed(); }

    /// Updates the problem pointer.
    void SetProblem(MultiAgentDecisionProcessInterface* p) { _m_problem = p; }

    /// Updates the horizon of the planning problem.
    virtual void SetHorizon(size_t horizon) { _m_horizon = horizon; }

    /// Initializes the random number generator (srand) to the stored seed.
    void InitSeed() const { srand(_m_seed); }

    //get (data) functions:    
    /// Returns the planning horizon.
    size_t GetHorizon() const { return(_m_horizon); }
    /// Returns the random seed stored.
    int GetSeed() const { return(_m_seed); }

    /// Return the number of agents.     
    size_t GetNrAgents() const { return(_m_problem->GetNrAgents()); }

    /// Maintains a agent index and returns the next one on calling */
    Index GetNextAgentIndex() {
        Index curIndex = _m_agentI;
        _m_agentI = (_m_agentI+1) % GetNrAgents();
        return(curIndex);
    }

    /// Get the problem pointer.
    const MultiAgentDecisionProcessInterface* GetProblem() const
        { return(_m_problem); }

    //other

    /// The function that performs the actual planning.
    virtual void Plan() = 0;

    /// Returns a pointer to the computed joint policy.
    virtual boost::shared_ptr<JointPolicy> GetJointPolicy() = 0;
};


#endif /* !_PLANNINGUNIT_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
