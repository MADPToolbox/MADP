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

/* Only include this header file once. */
#ifndef _AGENTBG_H_
#define _AGENTBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointBeliefSparse.h"
#include "AgentDelayedSharedObservations.h"
#include "QAV.h"

class PerseusBGPlanner;
class JointBeliefInterface;
class JointPolicyPureVector;
class BayesianGameIdenticalPayoff;

/**AgentBG represents an agent which uses a BG-based policy. */
class AgentBG : public AgentDelayedSharedObservations
{
private:    
    
    QAV<PerseusBGPlanner> *_m_QBGstationary;

    size_t _m_t;

    JointBeliefSparse _m_prevJB;
    
    BayesianGameIdenticalPayoff *_m_bgip;

    JointPolicyPureVector *_m_jpol;

    std::vector<Index> _m_oIs,_m_prevJoIs,_m_prevJaIs,_m_aIs;

    Index _m_jaIfirst;

    AlphaVector::BGPolicyIndex GetMaximizingBGIndex(const JointBeliefInterface &jb) const;
    Index GetMaximizingActionIndex(const JointBeliefInterface &jb) const;

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AgentBG(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
            QAV<PerseusBGPlanner> *QBG);

    /// Copy constructor.
    AgentBG(const AgentBG& a);

    /// Destructor.
    ~AgentBG();

    Index Act(Index oI, Index prevJoI);

    void ResetEpisode();

};


#endif /* !_AGENTBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
