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
#ifndef _BAYESIANGAMEIDENTICALPAYOFFINTERFACE_H_
#define _BAYESIANGAMEIDENTICALPAYOFFINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "BayesianGameBase.h"
#include "JointPolicyDiscretePure.h"

/** \brief BayesianGameIdenticalPayoffInterface provides an interface
 * for Bayesian Games with identical payoffs. */
class BayesianGameIdenticalPayoffInterface 
    : public BayesianGameBase
    
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BayesianGameIdenticalPayoffInterface()
        {};
        BayesianGameIdenticalPayoffInterface(
                    size_t nrAgents, 
                    const std::vector<size_t>& nrActions,  
                    const std::vector<size_t>& nrTypes)
        :
            BayesianGameBase(nrAgents, nrActions, nrTypes)
        {};

        BayesianGameIdenticalPayoffInterface& operator= (const BayesianGameIdenticalPayoffInterface& o)
        {
            if (this == &o) return *this;   // Gracefully handle self assignment

            BayesianGameBase::operator=(o);

            return *this;
        }
        
        //get (data) functions:
        /**Gets the utility for (for all agents) jtype, ja.*/
        virtual double GetUtility(const Index jtype, const Index ja) const = 0;
        /**Gets the utility for (for all agents) joint type corresponding to 
         * the individual type indices (indTypeIndices) and joint action
         * corresponding to individual action indices (indActionIndices).*/
        virtual double GetUtility(const std::vector<Index>& indTypeIndices, 
                const std::vector<Index>& indActionIndices ) const = 0;
        
        double ComputeValueJPol(const JointPolicyDiscretePure &jpolBG) const
        {
            double v=0.0;
            for (Index jtI=0; jtI<GetNrJointTypes(); jtI++)
                v += GetProbability(jtI) * GetUtility(jtI, jpolBG.GetJointActionIndex(jtI));
            return v;
        }
        /** Prints a description of this  entire BayesianGameIdenticalPayoff 
         * to a string.*/
        virtual std::string SoftPrint() const = 0; 
        /**Print this BayesianGameIdenticalPayoff to cout.*/
        virtual void Print() const
        { std::cout << SoftPrint();}

};


#endif /* !_BAYESIANGAMEIDENTICALPAYOFFINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
