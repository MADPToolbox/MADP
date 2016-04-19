/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "SimulationFactoredDecPOMDPDiscrete.h"
#include "JointPolicyDiscrete.h"

using namespace std;

SimulationFactoredDecPOMDPDiscrete::
SimulationFactoredDecPOMDPDiscrete(const PlanningUnitFactoredDecPOMDPDiscrete &pu, 
                              int nrRuns, int seed) : 
    SimulationDecPOMDPDiscrete(pu, nrRuns, seed),
    _m_puFactored(&pu)
{
}

SimulationFactoredDecPOMDPDiscrete::
SimulationFactoredDecPOMDPDiscrete(const PlanningUnitFactoredDecPOMDPDiscrete &pu, 
                              const ArgumentHandlers::Arguments &args) : 
    SimulationDecPOMDPDiscrete(pu, args),
    _m_puFactored(&pu)
{
}

//Destructor
SimulationFactoredDecPOMDPDiscrete::~SimulationFactoredDecPOMDPDiscrete()
{
}

void SimulationFactoredDecPOMDPDiscrete::Step(const std::vector<Index> &aIs, 
                                         unsigned int t,
                                         std::vector<Index> &sIs, 
                                         std::vector<Index> &oIs,
                                         double &r,
                                         double &sumR,
                                         double specialR) const
{
    vector<Index> sIs_suc;
    _m_puFactored->GetFDPOMDPD()->SampleSuccessorState(sIs,aIs,sIs_suc);

    _m_puFactored->GetFDPOMDPD()->SampleJointObservation(aIs,sIs_suc,oIs);
    r = _m_puFactored->GetFDPOMDPD()->GetReward(sIs,aIs) + specialR;

    // calc. the discounted reward
    sumR+=r*pow(_m_puFactored->GetDiscount(),static_cast<double>(t));

    if(GetVerbose())
        cout << "Simulation::RunSimulation "
             << "("
             << SoftPrintVector(sIs) << "," 
             << SoftPrintVector(aIs) << ","
             << SoftPrintVector(sIs_suc) << ") "
             << " jo " << SoftPrintVector(oIs) 
             << " r " << r << " sumR " << sumR << endl;
#if 0
             << "("
             << (_m_puFactored->GetFDPOMDPD()->GetState(sIs)->SoftPrintBrief())
             << ","
             << (_m_puFactored->GetJointAction(
                     _m_puFactored->IndividualToJointActionIndices(aIs)))
            ->SoftPrintBrief()
             << ","
             << (_m_puFactored->GetFDPOMDPD()->GetState(sIs_suc)->SoftPrintBrief())
             << ") (p " 
             << _m_puFactored->GetFDPOMDPD()->
            GetTransitionProbability(sIs,aIs,sIs_suc)
             << ") jo " << SoftPrintVector(oIs) << " "
             << (_m_puFactored->GetJointObservation(
                     _m_puFactored->IndividualToJointObservationIndices(oIs)))
            ->SoftPrintBrief()
             << " (p " 
             << _m_puFactored->GetFDPOMDPD()->
            GetObservationProbability(aIs,sIs_suc,oIs)
             << ") r " << r << " sumR " << sumR << endl;
#endif
    sIs = sIs_suc;
}

double
SimulationFactoredDecPOMDPDiscrete::RunSimulation(const JointPolicyDiscrete *jp) const
{
    size_t nrAgents=_m_puFactored->GetNrAgents();
    std::vector<Index> sIs(nrAgents);
    unsigned int t;
    double r,sumR=0;

    vector<Index> ohIs(nrAgents,0); // INITIAL_OHI does not exist
    
    _m_puFactored->GetFDPOMDPD()->SampleInitialState(sIs);

    if(GetVerbose())
        cout << "Simulation::RunSimulation " << endl
             << "Simulation::RunSimulation set initial state to " 
             << SoftPrintVector(sIs) << endl;

    vector<Index> aIs(nrAgents,INT_MAX),
        oIs(nrAgents,INT_MAX);

    for(t=0;t<_m_horizon;t++)
    {	
        jp->SampleJointActionVector(ohIs,aIs);

        Step(aIs, t, sIs, oIs, r, sumR, 0);
        
        /* action taken at ts=0,...,hor-1 - therefore only observation
         * histories at ts=0,...,hor-2 have successors.*/
        if(t < _m_horizon-1) 
        {
            for(Index i=0;i!=nrAgents;++i)
                ohIs[i] = _m_puFactored->GetSuccessorOHI(i, ohIs[i], oIs[i]);
        }
    }

    return(sumR);
}

SimulationResult
SimulationFactoredDecPOMDPDiscrete::RunSimulationsRandomActions() const
{
    SimulationResult result(_m_horizon,GetRandomSeed(),GetNrRuns());
    size_t nrAgents=_m_puFactored->GetNrAgents();
    vector<Index> sIs(nrAgents), aIs(nrAgents), oIs(nrAgents);
        unsigned int t;
    vector<size_t> nrAis=_m_puFactored->GetNrActions();
    double r,sumR=0;

    // Run the simulations
    int i;
    for(i=0;i<GetNrRuns();i++)
    {
        sumR=0;
        vector<Index> ohIs(nrAgents,0); // INITIAL_OHI does not exist
    
        _m_puFactored->GetFDPOMDPD()->SampleInitialState(sIs);
        for(t=0;t<_m_horizon;t++)
        {
           for(Index i=0;i!=nrAgents;++i)
                aIs[i]=static_cast<Index>(nrAis[i]*
                                          (rand() / (RAND_MAX + 1.0)));
            Step(aIs, t, sIs, oIs, r, sumR, 0);
        }
        result.AddReward(sumR);
    }

    return(result);
}
