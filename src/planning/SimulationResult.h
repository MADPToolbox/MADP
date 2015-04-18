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
#ifndef _SIMULATIONRESULT_H_
#define _SIMULATIONRESULT_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <string>
#include "Globals.h"

/** \brief SimulationResult stores the results from simulating a joint
 * policy, the obtained rewards in particular.
 *
 * At the moment only applies to DecPOMDPs. */
class SimulationResult 
{
private:    

    double _m_avg_reward;
    std::vector<double> _m_rewards;
    
    unsigned int _m_horizon;
    int _m_random_seed;
    
    unsigned int _m_nr_stored;

    void UpdateStatistics();
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    SimulationResult();

    /// Construct an object with certain parameters.
    SimulationResult(int horizon,int random_seed,int nrRuns);
    
    /// Destructor.
    ~SimulationResult();
    
    /// Add a sampled reward to the results set.
    void AddReward(double r);
    
    /// Get a sampled reward from the results set.
    double GetReward(Index i) const;

    /// Get the full set of stored reward samples.
    std::vector<double> GetRewards(void);

    /// The average of the stored reward samples.
    double GetAvgReward(void){ return(_m_avg_reward); }

    /// Save the reward samples to disk.
    void Save(std::string filename);

    /// Load reward samples from file.
    void Load(std::string filename);

    /// Print out the stored reward samples.
    void Print(void);

    /// Print out a summary.
    void PrintSummary(void);
};


#endif /* !_SIMULATIONRESULT_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
