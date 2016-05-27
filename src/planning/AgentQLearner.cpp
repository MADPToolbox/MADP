/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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

#include <iostream>
#include <limits>
#include <cfloat>
using namespace std;

#include "AgentQLearner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"

#define DEBUG_AgentQLearner 0
#define EPSILON 0.000000001 // replace with std::limits one?

AgentQLearner::AgentQLearner(const PlanningUnitDecPOMDPDiscrete *pu, Index id,
                             double initValue, double epsilon, double alpha, double gamma,
                             ExplorationT exploration, double temp) :
    AgentDecPOMDPDiscrete(pu, id),
    AgentFullyObservable(pu, id),
    _m_Q(pu->GetNrStates(),pu->GetNrJointActions(),initValue), // allocate Q table
    _m_firstAgent(this)
{
    _m_selJaI = INT_MAX;
    _m_prevSI = INT_MAX;
    _m_alpha = alpha;
    _m_gamma = gamma;
    _m_initValue = initValue;
    _m_exploration = exploration;
    _m_epsilon = epsilon;
    _m_temp = temp;
    _m_t = 0;
}

AgentQLearner::AgentQLearner(const AgentQLearner& a) :
    AgentDecPOMDPDiscrete(a),
    AgentFullyObservable(a),
    _m_Q(a._m_Q),
    _m_selJaI(a._m_selJaI),
    _m_prevSI(a._m_prevSI),
    _m_alpha(a._m_alpha),
    _m_gamma(a._m_gamma),
    _m_initValue(a._m_initValue),
    _m_exploration(a._m_exploration),
    _m_epsilon(a._m_epsilon),
    _m_temp(a._m_temp),
    _m_firstAgent(a._m_firstAgent),
    _m_t(a._m_t)
{
}

AgentQLearner::~AgentQLearner()
{
#if DEBUG_AgentQLearner
    cout << "Closed AgentQLearner with " << _m_Q.GetNrStates() << " states and "
         << _m_Q.GetNrActions() << " actions." << endl;
#endif
}

/**
 * This method updates the Q-values (prevSI,jaI) given the next state
 * sI and the received reward using the standard Bellman equation.
 */
void AgentQLearner::Learn(Index jaI, double r, Index sI, Index prevSI)
{
    // Perform one iteration of Q-learning in joint action & state space
    if(_m_t > 0 && isFirstAgent()) // we only need to learn once
    {
        // get the maximum value of the next state
        double maxNextState = getMaxState(sI);

        // reset if episode was ended
//        if(bEpisodeEnded)
//            maxNextState = 0.0;

#if DEBUG_AgentQLearner
        cout << "step: from state " << prevSI << " to state " << sI <<
                " after action " << jaI << " max. next state: " << maxNextState << " " <<
                _m_Q(prevSI,jaI) << " goes to ";
        cout << "a=" << _m_alpha << " oldq=" << _m_Q(prevSI,jaI) << " rew=" << r << " e=" << _m_epsilon << " max=" << maxNextState << " " << endl;
#endif
        _m_Q(prevSI,jaI) = (1.0 - _m_alpha) * _m_Q(prevSI,jaI) +
                _m_alpha*( r + _m_gamma * maxNextState );

#if DEBUG_AgentQLearner
        cout << _m_Q(prevSI,jaI) << " after reward " << r << endl;
#endif
    }
}

/**
 * This method returns the next action for state \a sI. Based on
 * the member variables either a greedy action or an exploration
 * action is taken. This is based on the exploration strategy (either
 * Boltzmann or e-greedy). When a greedy action is taken but multiple
 * Q-values have the same value, one of these is selected randomly.
 *
 * @param sI current state
 * @param r reward received at last iteration
 *
 * @return action for state sI
 */
Index AgentQLearner::Act(Index sI, Index /*joI*/, double r)
{
    Index jaInew;
    if(isFirstAgent()) // we only need to select joint action once
    {
        // perform an iteration of learning 
        if(_m_t > 0)
        {
            Learn(_m_selJaI, r, sI, _m_prevSI);
        }
        
        if( _m_exploration == EXPL_EGREEDY )
        {
            // e-greedy: random > epsilon choose best action
            if( (rand()/(RAND_MAX+1.0)) > _m_epsilon )
                jaInew = getGreedyAction(sI);
            else
                jaInew = getNonGreedyAction(sI);
        }
        else if( _m_exploration == EXPL_BOLTZMANN )
        {
            row_t row   = _m_Q.GetRow(sI);
            // calculate the normalization factor
            //FIXME parallelize
            double sum  = 0.0, sum2 = 0.0;
            for(unsigned j = 0; j < row.size(); ++j)
            {
                double b = exp( row(j) / _m_temp );
                sum += b;
            }

            double d = rand()/(RAND_MAX+1.0);
            unsigned j = 0;

            do
            {
                double b = exp( row(j) / _m_temp )/sum;
                sum2 += b;
                j++;
            } while( d > sum2 );
            j--;
            jaInew = (Index)j;
        }
        else
        {
            throw(E("AgentQLearner::Act error, sampling scheme not supported"));
            return INT_MAX;
        }
    }
    else
        jaInew = GetLastActionChosen();

    vector<Index> aIs=GetPU()->JointToIndividualActionIndices(jaInew);

    _m_selJaI = jaInew;
    _m_prevSI = sI;
    _m_t++;

#if DEBUG_AgentQLearner
    cout << GetIndex() << ": s " << sI << " ja "
         << jaInew << " aI " << aIs[GetIndex()] << endl;
#endif

    return(aIs[GetIndex()]);
}

/**
 * This method returns the highest Q-value in state \a sI. This
 * corresponds to the value associated with the getGreedyAction
 * action.
 *
 * \param sI state of which maximum Q-value will be returned
 *
 * \return maximum Q-value in sI
 */
double AgentQLearner::getMaxState(Index sI, list<Index>* actions) const
{
    if(actions)
        actions->clear();

    double dMax = -DBL_MAX;

    row_t row = _m_Q.GetRow(sI); // Q values for a specific state

    for(unsigned j = 0; j < row.size(); ++j)
    {
        if( std::abs( row(j) - dMax ) < EPSILON )
        {
            if(actions)
                actions->push_back(j);
        }
        else if( row(j) > dMax )
        {
            dMax = row(j);
            if(actions)
            {
                actions->clear();
                actions->push_back(j);
            }
        }
    }

    return dMax;
}

/**
 * This method returns the greedy action, corresponding to the action
 * with the highest Q-value, for the given state. When multiple Q-values
 * have the same optimum value, one of these actions is selected randomly.
 *
 * \param sI State for which greedy action has to be determined.
 *
 * \return action id
 */
Index AgentQLearner::getGreedyAction(Index sI) const
{
    list<Index> actions;
    double d = getMaxState(sI, &actions);

    // count number of other q-values that are equal to the highest value
    // normally count = 1 so iNr will be zero and the highest one is picked
    unsigned iNr = (unsigned)(rand()/(RAND_MAX+1.0)*actions.size());

    list<Index>::const_iterator iter = actions.begin();
    for(unsigned i = 0; i < iNr; i++)
        iter++;

#if DEBUG_AgentQLearner
    cout << "choose greedy action " << *iter << " in state " << sI
         << " with value " << d << endl;
#endif

    return *iter;
}

void AgentQLearner::ResetEpisode()
{
    _m_t=0;
}
