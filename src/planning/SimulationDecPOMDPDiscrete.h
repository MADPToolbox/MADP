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
#ifndef _SIMULATIONDECPOMDPDISCRETE_H_
#define _SIMULATIONDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "Simulation.h"
#include "SimulationResult.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "argumentHandlers.h"
#include "State.h"

class JointPolicyDiscrete;
class JointPolicyPureVectorForClusteredBG;
class AgentLocalObservations;
class AgentSharedObservations;
class AgentDelayedSharedObservations;
class AgentFullyObservable;

/**\brief SimulationDecPOMDPDiscrete simulates policies in
 * DecPOMDPDiscrete's.  */
class SimulationDecPOMDPDiscrete : public Simulation
{
private:    
    
    /// Pointer to the planning unit that generated the policy.
    const PlanningUnitDecPOMDPDiscrete* _m_pu;

    Index GetAction(const std::vector<AgentLocalObservations*> &agents,
                     Index i,
                     Index jaI, Index joI, double r, Index prevJoI,
                     Index sI, Index prevJaI, double &specialR) const;
    void PreActHook(const std::vector<AgentLocalObservations*> &agents,
                    Index jaI, Index joI, double r, Index prevJoI,
                    Index sI, Index prevJaI, Index ts) const {}

    Index GetAction(const std::vector<AgentSharedObservations*> &agents,
                     Index i,
                     Index jaI, Index joI, double r, Index prevJoI,
                     Index sI, Index prevJaI, double &specialR) const;
    void PreActHook(const std::vector<AgentSharedObservations*> &agents,
                    Index jaI, Index joI, double r, Index prevJoI,
                    Index sI, Index prevJaI, Index ts) const {}

    Index GetAction(const std::vector<AgentDelayedSharedObservations*> &agents,
                     Index i, Index jaI, Index joI, double r, Index prevJoI,
                     Index sI, Index prevJaI, double &specialR) const;
    void PreActHook(const std::vector<AgentDelayedSharedObservations*> &agents,
                    Index jaI, Index joI, double r, Index prevJoI,
                    Index sI, Index prevJaI, Index ts) const {}

    Index GetAction(const std::vector<AgentFullyObservable*> &agents, Index i,
                     Index jaI, Index joI, double r, Index prevJoI,
                     Index sI, Index prevJaI, double &specialR) const;
    void PreActHook(const std::vector<AgentFullyObservable*> &agents,
                    Index jaI, Index joI, double r, Index prevJoI,
                    Index sI, Index prevJaI, Index ts) const {}

    void Initialize();

    /// Perform one step of the simulation.
    void Step(Index jaI, unsigned int t, Index &sI,  Index &joI,
              double &r, double &sumR, double specialR) const;

    /// Simulate a run of a discrete joint policy.
    virtual double RunSimulation(const JointPolicyDiscrete* jp) const;

    /// Simulate a run of a JointPolicyPureVectorForClusteredBG.
    double RunSimulationClusteredBG(const JointPolicyPureVectorForClusteredBG* jp) const;

protected:
    
    size_t _m_horizon;

    bool _m_saveIntermediateResults;

    std::string _m_intermediateResultsFilename;

public:
    // Constructor, destructor and copy assignment.

    /// Constructor specifying the number of runs and the random seed.
    SimulationDecPOMDPDiscrete(const PlanningUnitDecPOMDPDiscrete &pu,
                               int nrRuns, int seed=illegalRandomSeed);

    /// Constructor which parses the command-line arguments.
    SimulationDecPOMDPDiscrete(const PlanningUnitDecPOMDPDiscrete &pu,
                               const ArgumentHandlers::Arguments &args);

    /// Destructor.
    ~SimulationDecPOMDPDiscrete();

    /// Run simulations using a particular discrete joint policy.
    SimulationResult
    RunSimulations(const JointPolicyDiscrete* jp) const;

    /// Run simulations using a particular discrete joint policy.
    SimulationResult
    RunSimulations(const boost::shared_ptr<JointPolicyDiscrete> &jp) const;

    /// Run simulations using a vector of SimulationAgent.
    template <class A>
    SimulationResult
    RunSimulations(const std::vector<A*> &agents) const
    {
        SimulationResult result(_m_horizon,GetRandomSeed(),GetNrRuns());

        // Run the simulations
        int i;
        for(i=0;i<GetNrRuns();i++)
        {
            Index jaI,sI,joI,prevJoI,prevJaI;
            int nr=agents.size(),i;
            std::vector<Index> aIs(nr);

            unsigned int h;
            double r=0,sumR=0,specialR;

            sI = _m_pu->GetDPOMDPD()->SampleInitialState();

            if(GetVerbose())
                std::cout << "Simulation::RunSimulation set initial state to " 
                     << sI << " "
                     << _m_pu->GetDPOMDPD()->GetState(sI)->SoftPrintBrief()
                     << " (avg reward so far " << result.GetAvgReward()
                          << ")" << std::endl;

            for(i=0;i<nr;++i)
                agents[i]->ResetEpisode();

            joI=INT_MAX;
            jaI=INT_MAX;
            prevJoI=INT_MAX;
            prevJaI=INT_MAX;
            for(h=0;h<_m_horizon;h++)
            {
                PreActHook(agents,jaI,joI,r,prevJoI,sI,prevJaI,h);
                specialR=0;
                // get the action for each particular agent
                for(i=0;i<nr;++i)
                    aIs[i]=GetAction(agents,i,jaI,joI,r,prevJoI,sI,prevJaI,
                                     specialR);
                jaI=_m_pu->IndividualToJointActionIndices(aIs);

                prevJoI=joI;
                prevJaI=jaI;
                Step(jaI, h, sI, joI, r, sumR, specialR);
            }
            
            result.AddReward(sumR);

            if(_m_saveIntermediateResults)
                result.Save(_m_intermediateResultsFilename);
        }
        
        return(result);
    }

    /// Indicate that intermediate should be stored to file named filename.
    void SaveIntermediateResults(std::string filename);


};


#endif /* !_SIMULATIONDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
