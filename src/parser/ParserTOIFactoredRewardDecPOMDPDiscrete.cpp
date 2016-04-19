/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ParserTOIFactoredRewardDecPOMDPDiscrete.h"

ParserTOIFactoredRewardDecPOMDPDiscrete::
ParserTOIFactoredRewardDecPOMDPDiscrete(
    TOIFactoredRewardDecPOMDPDiscrete
    *problem) :
    ParserTOIDecPOMDPDiscrete(problem),
    _m_problem(problem)
{
}

void ParserTOIFactoredRewardDecPOMDPDiscrete::
StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
              Index id)
{
    MultiAgentDecisionProcessDiscrete* madp = 
        _m_problem->GetIndividualMADPD(id);

    decpomdp->ExtractMADPDiscrete(madp);
    _m_problem->SetIndividualRewardModel(decpomdp->GetRewardModelPtr(), id);

    _m_problem->SetIndividualDecPOMDPD(decpomdp,id);
}
