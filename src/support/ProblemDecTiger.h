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

#ifndef _PROBLEMDECTIGER_H_
#define _PROBLEMDECTIGER_H_ 1

/* the include directives */

#include <iostream>
#include <string>

#include "DecPOMDPDiscrete.h"
#include "ActionDiscrete.h"
#include "JointActionDiscrete.h"
#include "ObservationDiscrete.h"
#include "JointObservationDiscrete.h"

/// ProblemDecTiger implements the DecTiger problem.
/** This class can be used as an alternative to parsing
 * \link dectiger.dpomdp \endlink.*/
class ProblemDecTiger : public DecPOMDPDiscrete
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
    
    enum jointAction_enum{
        LISTEN_LISTEN,
        LISTEN_OPENLEFT,
        LISTEN_OPENRIGHT,
        OPENLEFT_LISTEN,
        OPENLEFT_OPENLEFT,
        OPENLEFT_OPENRIGHT,
        OPENRIGHT_LISTEN,
        OPENRIGHT_OPENLEFT,
        OPENRIGHT_OPENRIGHT
    };

    enum observation_enum{
        HEARLEFT,
        HEARRIGHT
    };
    
    enum jointObservation_enum{
        HEARLEFT_HEARLEFT,
        HEARLEFT_HEARRIGHT,
        HEARRIGHT_HEARLEFT,
        HEARRIGHT_HEARRIGHT
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

protected:    
public:
    // constructors etc.
    /// Default constructor.
    ProblemDecTiger();
    /// Destructor. - let compiler generate destructor
    //virtual ~ProblemDecTiger();

};

#endif //! _PROBLEMDECTIGER_H_

// Local Variables: ***
// mode:c++ ***
// End: ***
