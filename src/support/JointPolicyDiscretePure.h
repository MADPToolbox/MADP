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
#ifndef _JOINTPOLICYDISCRETEPURE_H_
#define _JOINTPOLICYDISCRETEPURE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Interface_ProblemToPolicyDiscretePure.h"
#include "JointActionDiscrete.h"
#include "JointPolicyDiscrete.h"
#include "boost/shared_ptr.hpp"

class JointPolicyDiscretePure;
typedef boost::shared_ptr<JointPolicyDiscretePure> JPDP_sharedPtr;

class JointPolicyPureVector;
class PolicyDiscretePure;

/** \brief JointPolicyDiscretePure is represents a pure joint policy
 * for a discrete MADP.
 *
 * The number of pure joint policies as represented by this class is finite.
 * */
class JointPolicyDiscretePure : public JointPolicyDiscrete
{
    private:  

    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// default Constructor - constructs empty policy
        JointPolicyDiscretePure() {};
        /// (default) Constructor
        JointPolicyDiscretePure(const Interface_ProblemToPolicyDiscretePure* pu,
                                PolicyGlobals::PolicyDomainCategory idc );
        JointPolicyDiscretePure(const I_PtPDpure_constPtr &pu,
                                PolicyGlobals::PolicyDomainCategory idc );
        /// Copy constructor.
        JointPolicyDiscretePure(const JointPolicyDiscretePure& a);
        /// Destructor.
        virtual ~JointPolicyDiscretePure()
        {};
        /// Copy assignment operator
        virtual JointPolicyDiscretePure& operator= (const JointPolicyDiscretePure& o);

        //operators:

        //data manipulation (set) functions:     
        
        
        ///Randomly initialize the joint policy.
        virtual void RandomInitialization() = 0;
        ///Randomly initialize the policy for agentI.
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
        boost::shared_ptr<const Interface_ProblemToPolicyDiscretePure> 
            GetInterfacePTPDiscretePureShared() const
        {
            boost::shared_ptr<const Interface_ProblemToPolicyDiscrete> p =
                GetInterfacePTPDiscreteShared();
            boost::shared_ptr<const Interface_ProblemToPolicyDiscretePure> pp = 
                boost::dynamic_pointer_cast<const Interface_ProblemToPolicyDiscretePure>(p);
            return pp;
        }
        ///Sets the planning unit for this joint policy
        inline void SetInterfacePTPDiscretePure(
                Interface_ProblemToPolicyDiscretePure* pu);
        
        ///Sets the policy for agentI to domainI->aI.
        virtual void SetAction(Index agentI, Index domainI, Index aI) =0;

        ///Returns the jaI taken by this policy for joint domain index johI.
        virtual Index GetJointActionIndex(Index i) const = 0;
        ///Returns the jaI taken by this policy for joint domain index johI.
        virtual Index GetJointActionIndex(LIndex i) const = 0;
        ///Returns the action index for domainI for agent aI.
        virtual Index GetActionIndex(Index aI, Index domainI) const = 0;

        virtual boost::shared_ptr<JointPolicyPureVector> ToJointPolicyPureVector() const =0;
   
        /**\brief Returns the probability that the policy specifies joint
         * action a for domain index i.
         *
         * Required by JointPolicyDiscrete.
         */
        double GetJointActionProb( Index i, Index ja ) const;
        /**\brief Returns the probability that the policy specifies joint
         * action a for domain index i.
         *
         * Required by JointPolicyDiscrete.
         */
        double GetJointActionProb( LIndex i, Index ja ) const;
        /**\brief Returns the probability that the policy specifies 
         * action aI for domain index domI for agentI.
         *
         * Required by JointPolicyDiscrete.
         */
        double GetActionProb( Index agentI, Index domI, Index aI ) const;
                
        /// Returns a pointer to a copy of this class.
        virtual JointPolicyDiscretePure* Clone() const = 0;

        /// Prints a description of this JointPolicyPureVector to a string.
        virtual std::string SoftPrint() const = 0; 

        /// Prints a brief description to a string.
        virtual std::string SoftPrintBrief() const = 0; 
};


#endif /* !_JOINTPOLICYDISCRETEPURE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
