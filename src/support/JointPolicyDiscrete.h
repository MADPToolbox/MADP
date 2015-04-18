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
#ifndef _JOINTPOLICYDISCRETE_H_
#define _JOINTPOLICYDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <stdlib.h>
#include "Globals.h"
#include "JointPolicy.h"
#include "Interface_ProblemToPolicyDiscrete.h"
#include "Referrer.h"


using namespace PolicyGlobals;

class PolicyDiscrete;
/**\brief JointPolicyDiscrete is a class that represents a discrete joint
 * policy.
 *
 * A `discrete joint policy' is a policy for a discrete problem. I.e., the 
 * problem specifies, for each agent, a discrete domain for the policy and 
 * discrete actions.
 *
 * A JointPolicyDiscrete from discrete indices over the domain (typically 
 * action-observation histories or types) to (degenerate) probability 
 * distributions over indices over (joint) actions.
 *
 * This means that this class includes both pure and stochastic pure policies.
 *
 * */
class JointPolicyDiscrete : public JointPolicy
{
    private:   
        /// Maintains the PolicyDomainCategory. 
        PolicyGlobals::PolicyDomainCategory _m_indexDomCat;
        /// Interface that allows this policy to get some info on the problem
        const Interface_ProblemToPolicyDiscrete* _m_PTPD;
        I_PtPD_constPtr _m_PTPDshared;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// default constructor creates an empty policy
        JointPolicyDiscrete()
            : 
                JointPolicy(0),
                _m_PTPD(0)
        {};
        /// (default) Constructor
        JointPolicyDiscrete(const Interface_ProblemToPolicyDiscrete *pu,
                            PolicyGlobals::PolicyDomainCategory idc );
        JointPolicyDiscrete(const I_PtPD_constPtr &pu,
                            PolicyGlobals::PolicyDomainCategory idc );
        /// Copy constructor.
        JointPolicyDiscrete(const JointPolicyDiscrete& a);
        /// Destructor.
        virtual ~JointPolicyDiscrete()
        {};
        /// Assignment operator
        virtual JointPolicyDiscrete& operator= (const JointPolicyDiscrete& o);

        //operators:

        //data manipulation (set) functions:
        /**\brief sets the category of the domain over which the indices of
         * this policy are specified.
         */
        void SetPolicyDomainCategory(PolicyDomainCategory idc)
        {_m_indexDomCat = idc;};
        
        //get (data) functions:
        /**\brief returns the Category of the domain over which the indices of
         * this policy are specified.
         */
        PolicyDomainCategory GetPolicyDomainCategory() const
        {return (_m_indexDomCat);};


        /**\brief return pointer to individual policy of agent agI
         */
        virtual PolicyDiscrete* GetIndividualPolicyDiscrete(Index agI) const = 0;
        /**\brief Returns the probability that the policy of
         * agentI specifies action aI
         * for domain index domI.
         *
         * Implementations for pure policies clearly should return 0 or 1.
         */
        virtual double GetActionProb( Index agentI, 
                Index domI, Index aI ) const = 0;

        /**\brief Returns the probability that the policy specifies joint action
         * for joint domain index i.
         *
         * Implementations for pure policies clearly should return 0 or 1.
         */
        virtual double GetJointActionProb( Index i, Index ja ) const = 0;
        
        /**\brief Returns the probability that the policy specifies joint action
         * for joint domain index i.
         *
         * Implementations for pure policies clearly should return 0 or 1.
         */
        virtual double GetJointActionProb( LIndex i, Index ja ) const = 0;
        
        /**\brief Returns a sampled joint action.
         * i is the i-th joint domain index  */
        Index SampleJointAction( Index i ) const;
        /**\brief Returns a sampled joint action.
         * Is is the vector of domain indices  */
        Index SampleJointAction( const std::vector<Index>&  Is ) const;
        /**\brief Returns a sampled joint action.
         * i is the i-th joint domain index  */
        void SampleJointActionVector(Index i, std::vector<Index>& jaVec ) const;
        /**\brief Returns a sampled joint action.
         * Is is the vector of domain indices  */
        void SampleJointActionVector( 
                const std::vector<Index>& Is, std::vector<Index>& jaVec ) const;
        
        /**\brief return a pointer to the referred 
         * Interface_ProblemToPolicyDiscrete.
         */
        const Interface_ProblemToPolicyDiscrete* GetInterfacePTPDiscrete() const
        {
            if(_m_PTPDshared!=0)
                std::cerr << "Calling GetInterfacePTPDiscrete(), should use GetInterfacePTPDiscreteShared() instead"
                          << std::endl;

            if(_m_PTPD)
                return _m_PTPD;
            else
                return _m_PTPDshared.get();
        }
        /**\brief return a shared pointer to the referred 
         * Interface_ProblemToPolicyDiscrete.
         */
        boost::shared_ptr<const Interface_ProblemToPolicyDiscrete> GetInterfacePTPDiscreteShared() const
        {
            return _m_PTPDshared;
        }

        /**\brief sets the pointer to the Interface_ProblemToPolicyDiscrete.
         */
        void SetInterfacePTPDiscrete(const 
                Interface_ProblemToPolicyDiscrete* p)
        {_m_PTPD = p;}

        /**\brief sets the shared pointer to the Interface_ProblemToPolicyDiscrete.
         */
        void SetInterfacePTPDiscrete(I_PtPD_constPtr p)
        {_m_PTPDshared = p;}

        /// Returns the number of policy domain elements for agent agentI.
        size_t GetNrDomainElements(Index agentI) const
        { return(GetInterfacePTPDiscrete()->GetNrPolicyDomainElements(agentI,
                    GetPolicyDomainCategory() )); };

        /// Returns a pointer to a copy of this class.
        virtual JointPolicyDiscrete* Clone() const = 0;

};


#endif /* !_JOINTPOLICYDISCRETE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
