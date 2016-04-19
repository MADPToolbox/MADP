/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "AgentMDP.h"
#include <float.h>
#include <limits.h>
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

#define DEBUG_AgentMDP 0

AgentMDP::AgentMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                   const QTable &Q) :
    AgentDecPOMDPDiscrete(pu, id),
    AgentFullyObservable(pu, id),
    _m_Q(Q),
    _m_t(0)
{
}

AgentMDP::AgentMDP(const AgentMDP& a) :
    AgentDecPOMDPDiscrete(a),
    AgentFullyObservable(a),
    _m_Q(a._m_Q),
    _m_t(a._m_t)
{
}

//Destructor
AgentMDP::~AgentMDP()
{
}

Index AgentMDP::Act(Index sI, Index joI, double reward)
{
    Index jaInew=INT_MAX,aI;
    double q,v=-DBL_MAX;
    for(size_t a=0;a!=GetPU()->GetNrJointActions();++a)
    {
        q=_m_Q(sI,a);

        if(q>v)
        {
            v=q;
            jaInew=a;
        }
    }

    vector<Index> aIs=GetPU()->JointToIndividualActionIndices(jaInew);
    aI=aIs[GetIndex()];

    _m_t++;

#if DEBUG_AgentMDP
    cout << GetIndex() << ": s " << sI << " v " << v << " ja "
         << jaInew << " aI " << aI << endl;
#endif

    return(aI);
}

void AgentMDP::ResetEpisode()
{
    _m_t=0;
}
