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
#ifndef _FACTOREDSTATEAOHDISTRIBUTION_H_
#define _FACTOREDSTATEAOHDISTRIBUTION_H_ 1

/* the include directives */
#include "Globals.h"

class Scope;
class JointPolicyDiscretePure;
class FactoredStateDistribution;
/** \brief FactoredStateAOHDistribution is a class that represents 
 * a factored probability distribution over both states and 
 * action-observation histories. */
class FactoredStateAOHDistribution 
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// Destructor.
        virtual ~FactoredStateAOHDistribution(){};
/*
        /// (default) Constructor
        FactoredStateAOHDistribution();
        /// Copy constructor.
        FactoredStateAOHDistribution(const FactoredStateAOHDistribution& a);
        /// Copy assignment operator
        FactoredStateAOHDistribution& operator= (const FactoredStateAOHDistribution& o);
*/

        ///function that returns the prob. of the AOH of a subset of agents
        /**this function takes
         * \li \a agSC - the scope that specifies the subset of agents
         * \li \a aohI - a vector of indices of individual AOHs for each of the
         *               specified agents.
         *
         * Note: relT indicates that the individual AOH indices are relative
         * within the stage for which this distribution is.
         */
        virtual double GetAOHProb_relT(
                const Scope& agSc, 
                const std::vector<Index>& aohI) 
            const = 0;
        virtual double GetOHProb_relT(
                const Scope& agSc, 
                const std::vector<Index>& ohI)
            const = 0;
        virtual double GetXOHProb_relT(
                const Scope& sfSc, const std::vector<Index>& sfacI,
                const Scope& agSc, const std::vector<Index>& ohI
                ) const = 0;
        virtual double GetXProb(
                const Scope& sfSc, 
                const std::vector<Index>& sfValI)
            const=0;

        virtual void Update(const JointPolicyDiscretePure& pol)=0;


        virtual void InitializeFromISD( const FactoredStateDistribution * d)=0;
        virtual void SanityCheck()=0;

        virtual FactoredStateAOHDistribution *Clone() const = 0;
};


#endif /* !_FACTOREDSTATEAOHDISTRIBUTION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
