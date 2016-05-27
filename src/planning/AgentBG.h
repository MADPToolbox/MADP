/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
class PerseusBGNSPlanner;
class JointBeliefInterface;
class JointPolicyPureVector;
class BayesianGameIdenticalPayoff;

/**AgentBG represents an agent which uses a BG-based policy. */
class AgentBG : public AgentDelayedSharedObservations
{
private:    
    
    QAV<PerseusBGPlanner> *_m_QBGstationary;
    QAV<PerseusBGNSPlanner> *_m_QBGnonStationary;

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
    AgentBG(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
            QAV<PerseusBGNSPlanner> *QBG);

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
