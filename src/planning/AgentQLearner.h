/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

#ifndef _AGENT_QLEARNER_H_
#define _AGENT_QLEARNER_H_ 1

#include <list>

#include "AgentFullyObservable.h"
#include "QTable.h"

#include "Globals.h"

/// The possible exploration strategies when selecting an action.
enum ExplorationT
{
    EXPL_ILLEGAL,
    EXPL_BOLTZMANN,
    EXPL_EGREEDY
};

/** \brief AgentQLearner applies standard single-agent Q-learning in the joint action and state space.
 *
 **/
class AgentQLearner : public AgentFullyObservable
{
private:
    /// The tabular Q function to be learned.
    /// \todo consider sparse Q table
    QTable _m_Q;

    Index _m_selJaI;       //!< The selected action in the previous \c Act call.
    Index _m_prevSI;       //!< The state for which \c Act was last called.

    double _m_alpha;       //!< learning rate
    double _m_gamma;       //!< discount rate
    double _m_initValue;   //!< initial Q-value

    ExplorationT _m_exploration; //!< exploration strategy
    double _m_epsilon;     //!< greedy probability
    double _m_temp;        //!< boltzmann temperature

    /// The episode count.
    size_t _m_t;

    const AgentQLearner* _m_firstAgent;    //!< agent with id 0 for last action lookup

    Index GetLastActionChosen() const
        { return _m_firstAgent->_m_selJaI; }

public:

    // Constructor, destructor and copy assignment.
    /// Constructor
    AgentQLearner(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                  double initValue, double epsilon, double alpha, double gamma,
                  ExplorationT expl=EXPL_EGREEDY, double temp=0.4);

    /// Copy constructor.
    AgentQLearner(const AgentQLearner& a);

    /// Destructor.
    ~AgentQLearner();

    /// Update the internal Q table
    void Learn(Index jaI, double r, Index sI, Index prevSI);

    Index Act(Index sI, Index joI, double reward);

    double getMaxState(Index sI, std::list<Index>* actions = NULL) const;
    Index getGreedyAction(Index sI) const;
    Index getNonGreedyAction(Index sI) const
        { return (Index)(rand()/(RAND_MAX+1.0)*_m_Q.GetNrActions()); }

    void SetFirstAgent(const AgentQLearner* firstAgent)
        { _m_firstAgent = firstAgent; }

    bool isFirstAgent() const
        { return this == _m_firstAgent; }

    void updateEpsilon(double fract)
        { _m_epsilon *= fract; }

    void setTemp(double temp)
        { _m_temp = temp; }

    /// Return learned (infinite horizon) Q-Table
    QTable GetQTable() const
        { return _m_Q; }

    virtual void ResetEpisode();

};

#endif /* !_AGENT_QLEARNER_H_ */
