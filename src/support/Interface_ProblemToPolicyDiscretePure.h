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
#ifndef _INTERFACE_PROBLEMTOPOLICYDISCRETEPURE_H_
#define _INTERFACE_PROBLEMTOPOLICYDISCRETEPURE_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <cmath>
#include "Globals.h"
#include "Interface_ProblemToPolicyDiscrete.h"
#include "EOverflow.h"

class Interface_ProblemToPolicyDiscretePure;
typedef boost::shared_ptr<Interface_ProblemToPolicyDiscretePure> I_PtPDpure_sharedPtr;
typedef boost::shared_ptr<const Interface_ProblemToPolicyDiscretePure> I_PtPDpure_constPtr;

/** \brief Interface_ProblemToPolicyDiscretePure is an interface from
 * discrete problems to pure policies.
 *
 * I.e., it defines functions that must be implemented by a derived
 * (problem) class such that it can be used to construct a
 * JointPolicyPureVector / PolicyPureVector.
 * 
 * (the class (Joint)PolicyPureVector refers to objects implementing
 * this interface. ) */
class Interface_ProblemToPolicyDiscretePure : public Interface_ProblemToPolicyDiscrete
{
    private:       

    protected:
    
    public:
        /// Destructor.
        virtual ~Interface_ProblemToPolicyDiscretePure() {}

        /// Get the number of policies for an agent, given the policy's domain.
        LIndex GetNrPolicies(Index ag,
                             PolicyGlobals::PolicyDomainCategory cat, 
                             size_t depth=MAXHORIZON) const
        {        
#if 0 // don't use powl(), we want to detect overflow
            LIndex nr_pols = 
                static_cast<LIndex>(
                    powl( GetNrActions(ag),
                          GetNrPolicyDomainElements(ag,cat,depth) ));
#endif
            LIndex nrPols=1;
            for(size_t k=0;k!=GetNrPolicyDomainElements(ag,cat,depth);++k)
            {
                LIndex nrPolsOld=nrPols;
                nrPols*=GetNrActions(ag);
                if(nrPols<nrPolsOld)
                    throw(EOverflow("Interface_ProblemToPolicyDiscretePure::GetNrPolicies() overflow detected"));
            }
            return(nrPols);
        }

        /// Get the number of joint policies, given the policy's domain.
        LIndex GetNrJointPolicies(PolicyGlobals::PolicyDomainCategory cat, 
                                  size_t depth=MAXHORIZON) const
        {
            LIndex n=1;
            for(Index ag=0; ag < GetNrAgents(); ag++)
            {
                LIndex nOld=n;
                n *= GetNrPolicies(ag,cat,depth);
                if(n<nOld)
                    throw(EOverflow("Interface_ProblemToPolicyDiscretePure::GetNrJointPolicies() overflow detected"));
            }

            return(n);
        }

};


#endif /* !_INTERFACE_PROBLEMTOPOLICYDISCRETEPURE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
