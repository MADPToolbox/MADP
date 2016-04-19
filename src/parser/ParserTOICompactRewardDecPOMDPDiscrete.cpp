/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ParserTOICompactRewardDecPOMDPDiscrete.h"

ParserTOICompactRewardDecPOMDPDiscrete::
ParserTOICompactRewardDecPOMDPDiscrete(
    TOICompactRewardDecPOMDPDiscrete
    *problem) :
    ParserTOIDecPOMDPDiscrete(problem),
    _m_problem(problem)
{
}

void ParserTOICompactRewardDecPOMDPDiscrete::
StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
              Index id)
{
    MultiAgentDecisionProcessDiscrete* madp = 
        _m_problem->GetIndividualMADPD(id);

    decpomdp->ExtractMADPDiscrete(madp);
    _m_problem->SetIndividualRewardModel(decpomdp->GetRewardModelPtr(), id);

    _m_problem->SetIndividualDecPOMDPD(decpomdp,id);
}

void ParserTOICompactRewardDecPOMDPDiscrete::ParseRewards()
{
    size_t nrAgentsInInteraction=2;
    std::vector<size_t> nrStates(nrAgentsInInteraction,0),
        nrActions(nrAgentsInInteraction,0);
    for(Index i=0;i!=nrAgentsInInteraction;++i)
    {
        nrStates[i]=_m_problem->GetIndividualMADPD(i)->GetNrStates();
        nrActions[i]=_m_problem->GetIndividualMADPD(i)->GetNrJointActions();
    }
    ParseRewards(nrAgentsInInteraction,nrStates,nrActions);
}

