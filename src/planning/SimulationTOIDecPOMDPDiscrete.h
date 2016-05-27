/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _SIMULATIONTOIDECPOMDPDISCRETE_H_
#define _SIMULATIONTOIDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"
#include "PlanningUnitTOIDecPOMDPDiscrete.h"
#include "argumentHandlers.h"
#include "State.h"

class JointPolicyDiscrete;
class AgentTOIFullyObservableSynced;
class AgentTOIFullyObservableSyncedSpecialReward;

/**\brief SimulationTOIDecPOMDPDiscrete simulates policies in
 * TOIDecPOMDPDiscrete's.  */
class SimulationTOIDecPOMDPDiscrete : public SimulationDecPOMDPDiscrete
{
private:    
    
    /// Pointer to the planning unit that generated the policy.
    const PlanningUnitTOIDecPOMDPDiscrete* _m_puTOI;

    Index GetAction(const std::vector<AgentTOIFullyObservableSynced*> &agents,
                     Index i,
                     const std::vector<Index> &aIs,
                     const std::vector<Index> &oIs, 
                     double r, 
                     const std::vector<Index> &prevoIs,
                     const std::vector<Index> &sIs, 
                     const std::vector<Index> &prevaIs, 
                     double &specialR) const;
    void PreActHook(const std::vector<AgentTOIFullyObservableSynced*> &agents,
                    const std::vector<Index> &aIs,
                    const std::vector<Index> &oIs,
                    double r,
                    const std::vector<Index> &prevoIs,
                    const std::vector<Index> &sIs,
                    const std::vector<Index> &prevaIs,
                    Index ts) const;

    Index GetAction(const
                     std::vector<AgentTOIFullyObservableSyncedSpecialReward*>
                     &agents,
                     Index i,
                     const std::vector<Index> &aIs,
                     const std::vector<Index> &oIs, 
                     double r, 
                     const std::vector<Index> &prevoIs,
                     const std::vector<Index> &sIs, 
                     const std::vector<Index> &prevaIs, 
                     double &specialR) const;
    void PreActHook(const
                    std::vector<AgentTOIFullyObservableSyncedSpecialReward*>
                    &agents,
                    const std::vector<Index> &aIs,
                    const std::vector<Index> &oIs,
                    double r,
                    const std::vector<Index> &prevoIs,
                    const std::vector<Index> &sIs,
                    const std::vector<Index> &prevaIs,
                    Index ts) const;


    void Initialize();

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
    SimulationTOIDecPOMDPDiscrete(const PlanningUnitTOIDecPOMDPDiscrete &pu,
                                  int nrRuns, int seed=illegalRandomSeed);

    /// Constructor which parses the command-line arguments.
    SimulationTOIDecPOMDPDiscrete(const PlanningUnitTOIDecPOMDPDiscrete &pu,
                                  const ArgumentHandlers::Arguments &args);

    /// Destructor.
    ~SimulationTOIDecPOMDPDiscrete();

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
            int nr=agents.size(),i;
            std::vector<Index> sIs(nr);

            unsigned int h;
            double r=0,sumR=0,specialR;

            sIs = _m_puTOI->GetReferred()->SampleInitialStates();

            if(GetVerbose())
                std::cout << "Simulation::RunSimulation set initial state to " 
                          << SoftPrintVector(sIs) << " "
                          << _m_puTOI->GetReferred()->GetState(sIs)->
                             SoftPrintBrief()
                          << " (avg reward so far " << result.GetAvgReward()
                          << ")" << std::endl;

            for(i=0;i<nr;++i)
                agents[i]->ResetEpisode();

            std::vector<Index> aIs(nr,INT_MAX),
                oIs(nr,INT_MAX),
                prevoIs(nr,INT_MAX),
                prevaIs(nr,INT_MAX);

            for(h=0;h<_m_horizon;h++)
            {
                PreActHook(agents,aIs,oIs,r,prevoIs,sIs,prevaIs,h);
                specialR=0;
                // get the action for each particular agent
                for(i=0;i<nr;++i)
                    aIs[i]=GetAction(agents,i,aIs,oIs,r,prevoIs,sIs,prevaIs,
                                     specialR);

                prevoIs=oIs;
                prevaIs=aIs;
                Step(aIs, h, sIs, oIs, r, sumR, specialR);
            }
            
            result.AddReward(sumR);

            if(_m_saveIntermediateResults)
                result.Save(_m_intermediateResultsFilename);
        }
        
        return(result);
    }

};


#endif /* !_SIMULATIONTOIDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
