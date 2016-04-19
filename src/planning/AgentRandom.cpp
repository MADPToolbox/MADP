/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "AgentRandom.h"
#include <float.h>
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

#define DEBUG_AgentRandom 0

AgentRandom::AgentRandom(const PlanningUnitDecPOMDPDiscrete *pu, Index id) :
    AgentDecPOMDPDiscrete(pu, id),
    AgentFullyObservable(pu, id),
    AgentLocalObservations(pu, id)
{
}

AgentRandom::AgentRandom(const AgentRandom& a) :
    AgentDecPOMDPDiscrete(a),
    AgentFullyObservable(a),
    AgentLocalObservations(a)
{
}

//Destructor
AgentRandom::~AgentRandom()
{
}

Index AgentRandom::Act()
{
    vector<size_t> nrAis=GetPU()->GetNrActions();
    Index aI=static_cast<Index>(nrAis[GetIndex()]*
                                  (rand() / (RAND_MAX + 1.0)));
    return(aI);
}

void AgentRandom::ResetEpisode()
{
}
