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
#ifndef _BAYESIANGAME_H_
#define _BAYESIANGAME_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include "Globals.h"
#include "RewardModelMapping.h"
#include "BayesianGameBase.h"



/** \brief BayesianGame is a class that represents a general Bayesian
 * game in which each agent has its own utility function.
 *
 * This is a self contained class:
 * meaning that it does not depend on any Multi-agent decision problem or 
 * Planning unit. This implies that, in order to convert a time-step of a MADP 
 * Planning Unit to a Bayesian game, indices of observation(-action) histories 
 * have to be converted. This class uses its own indices.
 */
class BayesianGame : public BayesianGameBase 
{
    private:
        /**private bool to indicate whether this BG is initialized.
         * To access the BayesianGameBase initialized bool use:
         * BayesianGameBase::_m_initialized. */
        bool _m_initialized;

        /**Util functions - in general case 1 for each agent.
         * We use RewardModelMapping substituting joint type indices for state
         * indices. */
        std::vector<RewardModelMapping> _m_utilFuncs;
  
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        // (default) Constructor
        //BayesianGame();
        BayesianGame(size_t nrAgents, const std::vector<size_t> &nrActions,  
                     const std::vector<size_t> &nrTypes);
        /// Copy constructor.
        BayesianGame(const BayesianGame& a);

        //data manipulation (set) functions:
        
        /** \brief Sets the initialized status to b.
         *
         * When setting to true - checks are
         * performed to see if this is a consistent Bayesian Game.*/
        bool SetInitialized(bool b);

        /** \brief Sets the utility for agent, jtype, ja to u*/
        void SetUtility(const Index agent, const Index jtype, const Index ja, 
                const double u )
        {_m_utilFuncs[agent].Set(jtype,ja,u);}
        
        /// Prints out this BayesianGame.
        void Print() const;
};


#endif /* !_BAYESIANGAME_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
