/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "AgentQMDP.h"
#include <float.h>
#include <limits.h>
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

#define DEBUG_AgentQMDP 0

AgentQMDP::AgentQMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                     const QTable &Q) :
    AgentSharedObservations(pu,id),
    _m_Q(Q),
    _m_t(0)
{
}

AgentQMDP::AgentQMDP(const AgentQMDP& a) :
    AgentSharedObservations(a),
    _m_Q(a._m_Q),
    _m_t(a._m_t),
    _m_jb(a._m_jb),
    _m_prevJaI(a._m_prevJaI)
{
}

//Destructor
AgentQMDP::~AgentQMDP()
{
}

Index AgentQMDP::Act(Index joI)
{
    if(_m_t>0)
        _m_jb.Update(*GetPU()->GetDPOMDPD(),_m_prevJaI,joI);

    Index jaInew=INT_MAX,aI;
    double q,v=-DBL_MAX;
    for(size_t a=0;a!=GetPU()->GetNrJointActions();++a)
    {
        q=0;
        for(unsigned s=0;s!=GetPU()->GetNrStates();++s)
            q+=_m_jb.Get(s)*_m_Q(s,a);

//        if(q>(v+PROB_PRECISION))
        if(q>v)
        {
            v=q;
            jaInew=a;
        }
    }

    vector<Index> aIs=GetPU()->JointToIndividualActionIndices(jaInew);
    aI=aIs[GetIndex()];

    _m_prevJaI=jaInew;
    _m_t++;

#if DEBUG_AgentQMDP
    cout << GetIndex() << ": ";
    _m_jb.Print();
    cout << " v " << v << " ja " << jaInew << " aI " << aI << endl;
#endif

    return(aI);
}

void AgentQMDP::ResetEpisode()
{
    _m_t=0;
    JointBeliefInterface* jbi = GetPU()->GetNewJointBeliefFromISD();
    _m_jb.Set(jbi->Get());
    delete jbi;
    _m_prevJaI=INT_MAX;
}
