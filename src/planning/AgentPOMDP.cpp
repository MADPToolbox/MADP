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

#include "AgentPOMDP.h"
#include <float.h>
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointBeliefInterface.h"

using namespace std;

#define DEBUG_AgentPOMDP 0

AgentPOMDP::AgentPOMDP(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                       QAV<PerseusPOMDPPlanner> *QPOMDP) :
    AgentSharedObservations(pu,id),
    _m_QPOMDP(QPOMDP),
    _m_t(0)
{
    _m_jb=GetPU()->GetNewJointBeliefInterface();
}

AgentPOMDP::AgentPOMDP(const AgentPOMDP& a) :
    AgentSharedObservations(a),
    _m_QPOMDP(a._m_QPOMDP),
    _m_t(a._m_t),
    _m_prevJaI(a._m_prevJaI)
{
    _m_jb=GetPU()->GetNewJointBeliefInterface();
    *_m_jb=*a._m_jb;
}

//Destructor
AgentPOMDP::~AgentPOMDP()
{
    delete _m_jb;
}

Index AgentPOMDP::Act(Index joI)
{
    if(_m_t>0)
        _m_jb->Update(*GetPU()->GetDPOMDPD(),_m_prevJaI,joI);

    Index jaInew=INT_MAX;
    double q,v=-DBL_MAX;
    for(size_t a=0;a!=GetPU()->GetNrJointActions();++a)
    {
        q=_m_QPOMDP->GetQ(*_m_jb,a);
        if(q>v)
        {
            v=q;
            jaInew=a;
        }
    }

#if DEBUG_AgentPOMDP
    cout << GetIndex() << ": ";
    _m_jb.Print();
    cout << " v " << v << endl;
#endif

    vector<Index> aIs=GetPU()->JointToIndividualActionIndices(jaInew);
    
    _m_prevJaI=jaInew;
    _m_t++;

    return(aIs[GetIndex()]);
}

void AgentPOMDP::ResetEpisode()
{
    _m_t=0;
    *_m_jb=*GetPU()->GetNewJointBeliefFromISD(); // isa JointBeliefInterface*
    _m_prevJaI=INT_MAX;
}
