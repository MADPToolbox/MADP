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

