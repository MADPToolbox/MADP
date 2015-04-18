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

#ifndef _PROBLEMDECTIGERWITHCREAKS_H_
#define _PROBLEMDECTIGERWITHCREAKS_H_ 1

/* the include directives */

#include <iostream>
#include <string>

#include "DecPOMDPDiscrete.h"
#include "ActionDiscrete.h"
#include "JointActionDiscrete.h"
#include "ObservationDiscrete.h"
#include "JointObservationDiscrete.h"

/** ProblemDecTigerWithCreaks implements a variation of the DecTiger problem, in
 which an agent can hear whether the other agent has tried to open a
 door.  Defined in Gmytrasiewicz, P. J. and Doshi, P., "A Framework
 for Sequential Planning in Multi-Agent Settings", JAIR 24, 2005,
 pages 49-79. */
class ProblemDecTigerWithCreaks : public DecPOMDPDiscrete
{
private:

    /* aliases */
    typedef std::vector<ActionDiscrete> ActionIVec;
    typedef std::vector<ObservationDiscrete> ObservationIVec;

     /* constants */
    const size_t NUMBER_OF_STATES;
    const size_t NUMBER_OF_AGENTS;
    const size_t NUMBER_OF_OBSERVATIONS;
    const size_t NUMBER_OF_ACTIONS;

    enum state_enum{
        SLEFT,
        SRIGHT
    };

    enum action_enum{
        LISTEN,
        OPENLEFT,
        OPENRIGHT
    };
    
    enum observation_enum{
        GROWLLEFT_CREAKLEFT,
        GROWLLEFT_CREAKRIGHT,
        GROWLLEFT_SILENCE,
        GROWLRIGHT_CREAKLEFT,
        GROWLRIGHT_CREAKRIGHT,
        GROWLRIGHT_SILENCE
    };

    ///Construct all the Actions and actionSets (the vector _m_actionVecs).
    void ConstructActions();
    ///Construct all the observations and observation sets.
    void ConstructObservations();
    ///Fills the transition model with the tiger problem transitions.
    void FillTransitionModel();
    ///Fills the observation model with the tiger problem obs. probs.
    void FillObservationModel();
    ///Fills the reward model with the tiger problem rewards.
    void FillRewardModel();

    Index GetJointActionIndex(Index a0, Index a1) const;
    Index GetJointObsIndex(Index o0, Index o1) const;
    double GetObsProb(Index ja,Index s,Index agentI,Index oI) const;
    double GetRewardForAgent(Index s, Index ja, Index agentI) const;

protected:    
public:
    // constructors etc.
    /// Default constructor.
    ProblemDecTigerWithCreaks();
    /// Destructor.
    virtual ~ProblemDecTigerWithCreaks();

};

#endif //! _PROBLEMDECTIGERWITHCREAKS_H_

// Local Variables: ***
// mode:c++ ***
// End: ***
