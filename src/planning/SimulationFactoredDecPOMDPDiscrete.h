/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _SIMULATIONFACTOREDDECPOMDPDISCRETE_H_
#define _SIMULATIONFACTOREDDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "argumentHandlers.h"
#include "State.h"

class JointPolicyDiscrete;

/**\brief SimulationFactoredDecPOMDPDiscrete simulates policies in
 * FactoredDecPOMDPDiscrete's.  */
class SimulationFactoredDecPOMDPDiscrete : public SimulationDecPOMDPDiscrete
{
private:    
    
    /// Pointer to the planning unit that generated the policy.
    const PlanningUnitFactoredDecPOMDPDiscrete* _m_puFactored;

    /// Simulate a run of a discrete joint policy.
    double RunSimulation(const JointPolicyDiscrete* jp) const;

    /// Perform one step of the simulation.
    void Step(const std::vector<Index> &aIs, 
              unsigned int t,
              std::vector<Index> &sIs, 
              std::vector<Index> &oIs,
              double &r, double &sumR, double specialR) const;

protected:
    
public:
    // Constructor, destructor and copy assignment.

    /// Constructor specifying the number of runs and the random seed.
    SimulationFactoredDecPOMDPDiscrete(const PlanningUnitFactoredDecPOMDPDiscrete &pu,
                                  int nrRuns, int seed=illegalRandomSeed);

    /// Constructor which parses the command-line arguments.
    SimulationFactoredDecPOMDPDiscrete(const PlanningUnitFactoredDecPOMDPDiscrete &pu,
                                  const ArgumentHandlers::Arguments &args);

    /// Destructor.
    ~SimulationFactoredDecPOMDPDiscrete();

    SimulationResult RunSimulationsRandomActions() const;

};


#endif /* !_SIMULATIONFACTOREDDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
