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
#ifndef _POLICYDISCRETEPURE_H_
#define _POLICYDISCRETEPURE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Interface_ProblemToPolicyDiscretePure.h"
#include "PolicyDiscrete.h"

/** \brief PolicyDiscretePure is an abstract class that represents a
 * pure  policy for a discrete MADP.
 *
 * The number of pure  policies as represented by this class is finite.
 * Therefore it is possible
 * */
class PolicyDiscretePure : public PolicyDiscrete
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        PolicyDiscretePure(
                const Interface_ProblemToPolicyDiscretePure* pu,
                PolicyGlobals::PolicyDomainCategory idc,
                Index agentI
               );
        PolicyDiscretePure(
                const I_PtPD_constPtr &pu,
                PolicyGlobals::PolicyDomainCategory idc,
                Index agentI
               );
        /// Copy constructor.
        PolicyDiscretePure(const PolicyDiscretePure& a);
        /// Destructor.
        virtual ~PolicyDiscretePure()
        {}

        //operators:

        //data manipulation (set) functions:        
        
        /// Randomly initialize the  policy.
        //virtual void RandomInitialization() = 0;
        /// Randomly initialize the policy for agentI.
        virtual void RandomInitialization(Index agentI) = 0;
        
        //get (data) functions:

        /// Return pointer to the Interface_ProblemToPolicyDiscretePure.
        const Interface_ProblemToPolicyDiscretePure* 
            GetInterfacePTPDiscretePure() const
        {
            const Interface_ProblemToPolicyDiscrete* p =
                GetInterfacePTPDiscrete();
            const Interface_ProblemToPolicyDiscretePure* pp = 
                dynamic_cast<const Interface_ProblemToPolicyDiscretePure*>(p);
            return pp;
        }
    
        /** \brief Returns the jaI taken by this policy for  domain
         * index i.*/
        virtual Index GetActionIndex(Index i) const = 0;
   
        /**\brief Returns the probability that the policy specifies 
         * action aI for domain index i.
         *
         * Required by PolicyDiscrete.
         */
        double GetActionProb( Index i, Index aI ) const;

        /// Returns a pointer to a copy of this class.
        virtual PolicyDiscretePure* Clone() const = 0;
        
};


#endif /* !_POLICYDISCRETEPURE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
