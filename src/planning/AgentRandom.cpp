/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
