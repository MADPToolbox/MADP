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
#ifndef _BAYESIANGAMEIDENTICALPAYOFF_H_
#define _BAYESIANGAMEIDENTICALPAYOFF_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <math.h>
#include "Globals.h"
#include "RewardModelDiscreteInterface.h"
#include "BayesianGameBase.h"
#include "BayesianGameIdenticalPayoffInterface.h"

class JointPolicyDiscretePure;

class BayesianGameIdenticalPayoff;
typedef boost::shared_ptr<BayesianGameIdenticalPayoff> BGIP_sharedPtr;
typedef boost::shared_ptr<const BayesianGameIdenticalPayoff> BGIP_constPtr;

/**\brief BayesianGameIdenticalPayoff is a class that represents a
 * Bayesian game with identical payoffs. (there is just 1 util
 * function)
 *
 * This is a self contained class:
 * meaning that it does not depend on any Multi-agent decision problem or 
 * Planning unit. This implies that, in order to convert a time-step of a MADP 
 * Planning Unit to a Bayesian game, indices of observation(-action) histories 
 * have to be converted. This class uses its own indices.
 */
class BayesianGameIdenticalPayoff : 
    //virtual 
    //public BayesianGameBase,
    //virtual 
    public BayesianGameIdenticalPayoffInterface
{
    private:

        /**private bool to indicate whether this BG is initialized.
         * To access the BayesianGameBase initialized bool use:
         * BayesianGameBase::_m_initialized
         */
        bool _m_initialized;

        /**Util function - in identical payoff case we need only 1 util function
         * We use RewardModelMapping substituting joint type indices for state
         * indices. */
        RewardModelDiscreteInterface *_m_utilFunction;
            
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        // (default) Constructor
        BayesianGameIdenticalPayoff();
        BayesianGameIdenticalPayoff(size_t nrAgents, 
                                    const std::vector<size_t>& nrActions,  
                                    const std::vector<size_t>& nrTypes,
                                    bool useSparseRewardModel=false);
        /// Destructor.
        virtual ~BayesianGameIdenticalPayoff();
        //operators:

        /// Copy assignment operator
        BayesianGameIdenticalPayoff& operator= (const BayesianGameIdenticalPayoff& o);

        //data manipulation (set) functions:
        
        /**Sets the initialized status to b. When setting to true - checks are
         * performed to see if this is a consistent Bayesian Game.*/
        bool SetInitialized(bool b);
        /**Sets the utility for (for all agents) jtype, ja to u*/
        void SetUtility(const Index jtype, const Index ja, const double u )
        {
#if DEBUG_CHECKNANINF
            if(!std::isfinite(u))
                throw(E("BayesianGameIdenticalPayoff trying to set nan or inf utility"));
#endif
            _m_utilFunction->Set(jtype,ja,u);
        }
        /**Sets the utility for (for all agents) joint type corresponding to 
         * the individual type indices (indTypeIndices) and joint action
         * corresponding to individual action indices (indActionIndices).*/
        void SetUtility(const std::vector<Index>& indTypeIndices, 
                const std::vector<Index>& indActionIndices,const double u )
        {
#if DEBUG_CHECKNANINF
            if(!std::isfinite(u))
                throw(E("BayesianGameIdenticalPayoff trying to set nan or inf utility"));
#endif
            _m_utilFunction->Set(
                IndividualToJointTypeIndices(indTypeIndices),
                IndividualToJointActionIndices(indActionIndices),
                u);
        }
        
        //get (data) functions:
        /**Gets the utility for (for all agents) jtype, ja.*/
        double GetUtility(const Index jtype, const Index ja) const
            {return(_m_utilFunction->Get(jtype,ja));}
        /**Gets the utility for (for all agents) joint type corresponding to 
         * the individual type indices (indTypeIndices) and joint action
         * corresponding to individual action indices (indActionIndices).*/
        double GetUtility(const std::vector<Index>& indTypeIndices, 
                const std::vector<Index>& indActionIndices ) const
        {return(_m_utilFunction->Get(
                IndividualToJointTypeIndices(indTypeIndices),
                IndividualToJointActionIndices(indActionIndices) ));}
        
        /**\brief evaluates the value of a joint policy.*/
        virtual double ComputeValueJPol(const JointPolicyDiscretePure & jpolBG) const;

        /** Prints a description of this  entire BayesianGameIdenticalPayoff 
         * to a string.*/
        std::string SoftPrint() const; 
        /**Print this BayesianGameIdenticalPayoff to cout.*/
        void Print() const
        { std::cout << SoftPrint();}
        /**Prints the utilities for jtype*/
        std::string SoftPrintUtilForJointType(Index jtype) const;
        /**Prints the utilities for jtype*/
        void PrintUtilForJointType(Index jtype) const
        {std::cout << SoftPrintUtilForJointType(jtype);}

        static void Save(const BayesianGameIdenticalPayoff &bg,
                         const std::string &filename);
        static void SaveTextFormat(const BayesianGameIdenticalPayoff &bg,
                                   const std::string &filename);

        ///Loads a BG from file
        /**Note that it returns the BG by value, so the copy assignment
         * operator should be working. Also, this may be less suitable
         * for large BGs.
         */
        static BayesianGameIdenticalPayoff Load(const std::string &filename);
        static BayesianGameIdenticalPayoff LoadTextFormat(const std::string &filename);

        ///Generates a random BG with identical payoffs.
        /**Note that it returns the BG by value, so the copy assignment
         * operator should be working. Also, this may be less suitable
         * for large BGs.
         */
        static BayesianGameIdenticalPayoff GenerateRandomBG(
            size_t nrAgents,
            std::vector<size_t> acs,
            std::vector<size_t> obs
            );
};


#endif /* !_BAYESIANGAMEIDENTICALPAYOFF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
