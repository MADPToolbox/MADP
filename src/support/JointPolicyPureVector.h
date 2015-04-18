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
#ifndef _JOINTPOLICYPUREVECTOR_H_
#define _JOINTPOLICYPUREVECTOR_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include "Globals.h"
#include "JointPolicyDiscretePure.h"
#include "JPolComponent_VectorImplementation.h"
#include "boost/shared_ptr.hpp"

class JointPolicyPureVector;
typedef boost::shared_ptr<JointPolicyPureVector> JPPV_sharedPtr;

/// JointPolicyPureVector represents a discrete pure joint policy.
/** 
 * Each of the individual policies is a PolicyPureVector - a
 * deterministic mapping from observations to actions implemented by a
 * tree. This class acts as a wrapper. It contains a vector with
 * pointers to indiv. policies. 
 *
 * we delete the PolicyPureVectors pointed to: this means that we should not 
 * be able to add indiv. policies without copying them!!! 
 *
 * */
class JointPolicyPureVector  :  
    //virtual <- this leads to pain in the ass...
    public JointPolicyDiscretePure,
    private JPolComponent_VectorImplementation // use private inheritance for composition

{
    private:

    /** \brief Private function that deletes all the individual
     * policies pointed to.*/
    void ClearIndividualPolicies();

    protected:
        std::vector<PolicyPureVector*> _m_indivPols_PolicyPureVector;

    public:
        // Constructor, destructor and copy assignment.

        /**\brief Constructor. 
         *
         * Associates a problem (typically a planning unit) with the joint
         * policy.  Information regarding the problem is used to construct a
         * joint policy of the proper shape.
         * 
         * Use the default PolicyDomainCategory defined by pu.
         * */
        JointPolicyPureVector(const Interface_ProblemToPolicyDiscretePure* pu)
          :
          JointPolicyDiscretePure( pu , pu->GetDefaultIndexDomCat()  ),
          JPolComponent_VectorImplementation(pu,  pu->GetDefaultIndexDomCat() )
        {};
        /**\brief Constructor. 
         *
         * Associates a problem (typically a planning unit) with the joint
         * policy.  Information regarding the problem is used to construct a
         * joint policy of the proper shape.
         *
         * idc is the PolicyDomainCategory over which the policy is specified.
         *
         * */
        JointPolicyPureVector(const Interface_ProblemToPolicyDiscretePure* pu,
                              PolicyGlobals::PolicyDomainCategory idc )
            :
            JointPolicyDiscretePure(pu, idc),
            JPolComponent_VectorImplementation(pu, idc)
        {};

        /**\brief Constructor. 
         *
         * Associates a problem (typically a planning unit) with the joint
         * policy.  Information regarding the problem is used to construct a
         * joint policy of the proper shape.
         * 
         * Use the default PolicyDomainCategory defined by pu.
         * */
        JointPolicyPureVector(I_PtPDpure_constPtr pu)
          :
          JointPolicyDiscretePure( pu , pu->GetDefaultIndexDomCat()  ),
          JPolComponent_VectorImplementation(pu,  pu->GetDefaultIndexDomCat() )
        {};
        /**\brief Constructor. 
         *
         * Associates a problem (typically a planning unit) with the joint
         * policy.  Information regarding the problem is used to construct a
         * joint policy of the proper shape.
         *
         * idc is the PolicyDomainCategory over which the policy is specified.
         *
         * */
        JointPolicyPureVector(I_PtPDpure_constPtr pu,
                              PolicyGlobals::PolicyDomainCategory idc )
            :
            JointPolicyDiscretePure(pu, idc),
            JPolComponent_VectorImplementation(pu, idc)
        {};

        /// Copy constructor.
        JointPolicyPureVector(const JointPolicyPureVector& a)
            :
            JointPolicyDiscretePure(a)
            , JPolComponent_VectorImplementation(a)
        {};
#if 0 // this can only lead to problems...
        /// Copy constructor from base class a
        JointPolicyPureVector(const JointPolicyDiscretePure& a)
            :
            JointPolicyDiscretePure(a),
            JPolComponent_VectorImplementation( 
                dynamic_cast<const JointPolicyPureVector&>( a ) )
        {};
#endif
        /// Copy constructor from base class a
        JointPolicyPureVector(const JointPolicyDiscretePure& a1, 
                const JPolComponent_VectorImplementation& a2)
            :
            JointPolicyDiscretePure(a1),
            JPolComponent_VectorImplementation( a2 )
        {};
    private:
    public:
        /// Destructor.
        virtual ~JointPolicyPureVector() {};
        /**The copy assignment operator - makes deep copies using 
         * PolicyPureVector::operator= which makes deep copies.*/
        JointPolicyPureVector& operator= (const JointPolicyPureVector& jp);
        /**The copy assignment operator - makes deep copies using 
         * PolicyPureVector::operator= which makes deep copies.*/
        JointPolicyPureVector& operator= (const JointPolicyDiscretePure& jp);

        bool operator< (const JointPolicy& o) const
        {
            const JointPolicyPureVector& p = dynamic_cast<const JointPolicyPureVector&>(o);
            return (*this) < p;
        }
        bool operator< (const JointPolicyPureVector& o) const
        { return (GetIndex() < o.GetIndex()) ; }
            
        /// Sets the depth of the joint policy.
        /** Also sets the depth of each of the invididual policies. */
        void SetDepth(size_t d)
        {
            if(d==MAXHORIZON)
                throw(E("JointPolicyPureVector can only be used for finite horizons."));
            JointPolicy::SetDepth(d);
            JPolComponent_VectorImplementation::SetDepthForIndivPols(d);
        };

        /// Prints a description of this JointPolicyPureVector to a string.
        std::string SoftPrint() const; 
        /// Prints a brief description to a string.
        std::string SoftPrintBrief() const; 
        void PrintBrief() const
        { std::cout << SoftPrintBrief();}
        
        /**\brief Convert this joint policy to a JointPolicyPureVector.
         */
        JPPV_sharedPtr ToJointPolicyPureVector() const;
        
    //relay the following functions to the JPolComponent_VectorImplementation:
        
        ///Performs increment. See Increment().
        bool operator++ ()   {    return( this->Increment() );     };

        bool Increment(Index agentI)
        { return this->JPolComponent_VectorImplementation::Increment(agentI); }
        bool Increment()
        { return this->JPolComponent_VectorImplementation::Increment(); }
        
        /// Get the index of this joint policy.
        LIndex GetIndex() const
        {   return this->JPolComponent_VectorImplementation::GetIndex(); }
        /// Sets the index of this joint policy.Updates the policies represented
        void SetIndex(LIndex i)
        {   this->JPolComponent_VectorImplementation::SetIndex(i); }
        

        ///Returns the jaI taken by this policy for joint domain index johI.
        inline Index GetJointActionIndex(Index i) const
        { return JPolComponent_VectorImplementation::GetJointActionIndex(i); }
        inline Index GetJointActionIndex(LIndex i) const
        { return JPolComponent_VectorImplementation::GetJointActionIndex(i); }
        ///Returns the action index for domainI for agent aI.
        Index GetActionIndex(Index agI , Index domainI) const
        { return JPolComponent_VectorImplementation::GetActionIndex(agI , domainI);}

        PolicyDiscrete* GetIndividualPolicyDiscrete(Index i) const
        { return JPolComponent_VectorImplementation::GetIndividualPolicyDiscrete(i) ;}
        /// Returns a reference to the vector of pointers to individual policies
        /** This function should be used to manipulate individual policies.
         */
        std::vector<PolicyPureVector*>& GetIndividualPolicies()
        {return this->JPolComponent_VectorImplementation::GetIndividualPolicies();}

        /// Initialize the joint policy to the first joint policy.
        /** This is the joint policy that specifies action 0 --the
         *  first action-- for all observation histories.*/
        void ZeroInitialization()
        { JPolComponent_VectorImplementation::ZeroInitialization();}
        /// Initialize the policy of agentI to the first policy.
        /** This is the policy that specifies action 0 --the first
         *  action-- for all observation histories.*/
        void ZeroInitialization(Index i)
        { JPolComponent_VectorImplementation::ZeroInitialization(i);}
        ///Randomly initialize the joint policy.
        void RandomInitialization()
        { JPolComponent_VectorImplementation::RandomInitialization();}
        ///Randomly initialize the policy for agentI.
        void RandomInitialization(Index i)
        { JPolComponent_VectorImplementation::RandomInitialization(i);}
        void SetAction(Index agentI, Index domainI, Index aI)
        { JPolComponent_VectorImplementation::SetAction(agentI, domainI, aI);}

        /// Returns a pointer to a copy of this class.
        virtual JointPolicyPureVector* Clone() const
        { return new JointPolicyPureVector(*this); }


      
};


#endif /* !_JOINTPOLICYPUREVECTOR_H_ */


// Local Variables: ***
// mode:c++ ***
;// End: ***
