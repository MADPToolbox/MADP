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
#ifndef _INTERFACE_PROBLEMTOPOLICYDISCRETE_H_
#define _INTERFACE_PROBLEMTOPOLICYDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <cmath>
#include "Globals.h"
#include "PolicyGlobals.h"
#include "boost/shared_ptr.hpp"

class Interface_ProblemToPolicyDiscrete;
typedef boost::shared_ptr<Interface_ProblemToPolicyDiscrete> I_PtPD_sharedPtr;
typedef boost::shared_ptr<const Interface_ProblemToPolicyDiscrete> I_PtPD_constPtr;

/** \brief Interface_ProblemToPolicyDiscrete is an interface from
 * discrete problems to policies.
 *
 * I.e., it defines functions that must be implemented by a derived
 * (problem) class such that it can be used to construct a
 * JointPolicyPureVector / PolicyPureVector.
 * 
 * (the class (Joint)PolicyPureVector refers to objects implementing
 * this interface. ) */
class Interface_ProblemToPolicyDiscrete 
{
    private:       

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        Interface_ProblemToPolicyDiscrete()
        {}
        /// Destructor.
        virtual ~Interface_ProblemToPolicyDiscrete() {}

        /// Check whether certain index conversions are cached.
        /** \deprecated policy should not need to now!
         */
        virtual bool AreCachedJointToIndivIndices(
                const PolicyGlobals::PolicyDomainCategory pdc) const = 0;

        /// Get the number of agents involved in this problem.
        virtual size_t GetNrAgents() const = 0;
        /// Get the number of joint actions.
        size_t GetNrJointActions() const
        {
            size_t nrJA = 1;
            for(Index i=0; i < GetNrAgents(); i++)
                nrJA *= GetNrActions(i);
            return(nrJA);
        }

        /// Get the number of invididual actions of a particular agent.
        virtual size_t GetNrActions(Index agentI) const = 0;   
        /// Get the number of elements in the domain of an agent's policy.
        virtual size_t GetNrPolicyDomainElements(Index agentI,
                                                 PolicyGlobals::PolicyDomainCategory cat,
                                                 size_t depth=MAXHORIZON) const = 0;

        /// Converts individual action indices to a joint action index.
        /** indivIndices is an array of size GetNrAgents()
         */
        virtual Index IndividualToJointActionIndices(
                const Index* indivIndices) const = 0;
        /// Converts individual action indices to a joint action index.
        virtual Index IndividualToJointActionIndices(
                const std::vector<Index>& indivIndices) const = 0;

        /// Return the default PolicyDomainCategory for the problem.
        virtual PolicyGlobals::PolicyDomainCategory GetDefaultIndexDomCat()
            const = 0;

        /// Converts joint indices to individual policy domain element indices.
        /**A function that should be implemented by a derived class,
         * if this derived class caches index vectors of the policy
         * domains elements corresponding to a joint index. See
         * AreCachedJointToIndivIndices()
         */
        virtual std::vector<Index> JointToIndividualPolicyDomainIndices(Index jdI,
                PolicyGlobals::PolicyDomainCategory cat) const = 0;
        /// Converts individual policy domain element indices to joint indices.
        /**A function that should be implemented by a derived class,
         * if this derived class caches index vectors of the policy
         * domains elements corresponding to a joint index. See
         * AreCachedJointToIndivIndices()
         */
        virtual const std::vector<Index>& JointToIndividualPolicyDomainIndicesRef( 
                Index jdI, PolicyGlobals::PolicyDomainCategory cat) const = 0;
        /**Virtual function that has to be implemented by derived class.
         * This should return a string describing element dIndex of the domain
         * (e.g. an observation history) of agent agentI.*/
        virtual std::string SoftPrintPolicyDomainElement(Index agentI, Index dIndex,
                PolicyGlobals::PolicyDomainCategory cat ) 
            const=0;
        /**Virtual function that has to be implemented by derived class.
         * This should return a string describing action actionI 
         * of agent agentI.*/
        virtual std::string SoftPrintAction(Index agentI, Index actionI) const =0;

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_INTERFACE_PROBLEMTOPOLICYDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
