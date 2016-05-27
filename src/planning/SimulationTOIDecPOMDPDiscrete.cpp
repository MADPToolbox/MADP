/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "SimulationTOIDecPOMDPDiscrete.h"
#include "AgentTOIFullyObservableSynced.h"
#include "AgentTOIFullyObservableSyncedSpecialReward.h"

using namespace std;

SimulationTOIDecPOMDPDiscrete::
SimulationTOIDecPOMDPDiscrete(const PlanningUnitTOIDecPOMDPDiscrete &pu, 
                              int nrRuns, int seed) : 
    SimulationDecPOMDPDiscrete(pu, nrRuns, seed),
    _m_puTOI(&pu)
{
}

SimulationTOIDecPOMDPDiscrete::
SimulationTOIDecPOMDPDiscrete(const PlanningUnitTOIDecPOMDPDiscrete &pu, 
                              const ArgumentHandlers::Arguments &args) : 
    SimulationDecPOMDPDiscrete(pu, args),
    _m_puTOI(&pu)
{
}

//Destructor
SimulationTOIDecPOMDPDiscrete::~SimulationTOIDecPOMDPDiscrete()
{
}

void SimulationTOIDecPOMDPDiscrete::Step(const std::vector<Index> &aIs, 
                                         unsigned int t,
                                         std::vector<Index> &sIs, 
                                         std::vector<Index> &oIs,
                                         double &r,
                                         double &sumR,
                                         double specialR) const
{
    vector<Index> sIs_suc=_m_puTOI->GetReferred()->
        SampleSuccessorState(sIs,aIs);

    oIs=_m_puTOI->GetReferred()->SampleJointObservation(aIs,sIs_suc);
    r = _m_puTOI->GetReferred()->GetReward(sIs,aIs) + specialR;

    // calc. the discounted reward
    sumR+=r*pow(_m_puTOI->GetDiscount(),static_cast<double>(t));
    
    if(GetVerbose())
        cout << "Simulation::RunSimulation "
#if 0
             << "("
             << SoftPrintVector(sIs) << "," 
             << SoftPrintVector(aIs) << ","
             << SoftPrintVector(sIs_suc) << ") "
#endif
             << "("
             << (_m_puTOI->GetReferred()->GetState(sIs)->SoftPrintBrief())
             << ","
             << (_m_puTOI->GetJointAction(
                     _m_puTOI->IndividualToJointActionIndices(aIs)))
            ->SoftPrintBrief()
             << ","
             << (_m_puTOI->GetReferred()->GetState(sIs_suc)->SoftPrintBrief())
             << ") (p " 
             << _m_puTOI->GetReferred()->
            GetTransitionProbability(sIs,aIs,sIs_suc)
             << ") jo " << SoftPrintVector(oIs) << " "
             << (_m_puTOI->GetJointObservation(
                     _m_puTOI->IndividualToJointObservationIndices(oIs)))
            ->SoftPrintBrief()
             << " (p " 
             << _m_puTOI->GetReferred()->
            GetObservationProbability(aIs,sIs_suc,oIs)
             << ") r " << r << " sumR " << sumR << endl;

    sIs = sIs_suc;
}

Index SimulationTOIDecPOMDPDiscrete::GetAction(
    const std::vector<AgentTOIFullyObservableSynced*> &agents,
    Index i,
    const std::vector<Index> &aIs,
    const std::vector<Index> &oIs, 
    double r, 
    const std::vector<Index> &prevoIs,
    const std::vector<Index> &sIs, 
    const std::vector<Index> &prevaIs, 
    double &specialR) const
{
    return(agents[i]->Act(sIs,oIs,r));
}

void SimulationTOIDecPOMDPDiscrete::PreActHook(
    const std::vector<AgentTOIFullyObservableSynced*> &agents,
    const std::vector<Index> &aIs,
    const std::vector<Index> &oIs,
    double r,
    const std::vector<Index> &prevoIs,
    const std::vector<Index> &sIs,
    const std::vector<Index> &prevaIs,
    Index ts) const
{
    for(Index j=0;j<agents.size();++j)
        agents[j]->Sync(sIs,oIs,r);
}

Index SimulationTOIDecPOMDPDiscrete::GetAction(
    const std::vector<AgentTOIFullyObservableSyncedSpecialReward*> &agents,
    Index i,
    const std::vector<Index> &aIs,
    const std::vector<Index> &oIs, 
    double r, 
    const std::vector<Index> &prevoIs,
    const std::vector<Index> &sIs, 
    const std::vector<Index> &prevaIs, 
    double &specialR) const
{
    Index a=agents[i]->Act(sIs,oIs,r);
    specialR+=agents[i]->GetSpecialReward();
    return(a);
}

void SimulationTOIDecPOMDPDiscrete::PreActHook(
    const std::vector<AgentTOIFullyObservableSyncedSpecialReward*> &agents,
    const std::vector<Index> &aIs,
    const std::vector<Index> &oIs,
    double r,
    const std::vector<Index> &prevoIs,
    const std::vector<Index> &sIs,
    const std::vector<Index> &prevaIs,
    Index ts) const
{
    for(Index j=0;j<agents.size();++j)
        agents[j]->Sync(sIs,oIs,r);
}
