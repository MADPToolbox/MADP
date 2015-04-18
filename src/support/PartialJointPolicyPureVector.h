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
#ifndef _PARTIALJOINTPOLICYPUREVECTOR_H_
#define _PARTIALJOINTPOLICYPUREVECTOR_H_ 1

/* the include directives */
#include "Globals.h"
#include "JPolComponent_VectorImplementation.h"
#include "PartialJointPolicyDiscretePure.h"
#include "boost/shared_ptr.hpp"

class PartialJointPolicyPureVector;
typedef boost::shared_ptr<PartialJointPolicyPureVector> PJPPV_sharedPtr;

/** \brief PartialJointPolicyPureVector implements a
 * PartialJointPolicy using a mapping of history indices to
 * actions. */
class PartialJointPolicyPureVector :
//    public JointPolicyPureVector , //<- don't use mutliple inheritance for code reuse..
    public PartialJointPolicyDiscretePure,
    private JPolComponent_VectorImplementation // use private inheritance for composition
{
    private:    
        
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        PartialJointPolicyPureVector(
                const Interface_ProblemToPolicyDiscretePure * pu,
                PolicyGlobals::PolicyDomainCategory idc,
                double pastReward = 0.0,
                size_t depth = MAXHORIZON);

        PartialJointPolicyPureVector(
                const I_PtPDpure_constPtr &pu,
                PolicyGlobals::PolicyDomainCategory idc,
                double pastReward = 0.0,
                size_t depth = MAXHORIZON);

        /// Copy constructor.
        PartialJointPolicyPureVector(const PartialJointPolicyPureVector& a)
            :
            PartialJointPolicyDiscretePure(a)
            , JPolComponent_VectorImplementation(a)
        {};
        PartialJointPolicyPureVector(const PartialJointPolicyDiscretePure& a)
            :
            PartialJointPolicyDiscretePure(a),
            JPolComponent_VectorImplementation( 
                dynamic_cast<const PartialJointPolicyPureVector&>( a ) )
        {};


        /// Destructor.
        virtual ~PartialJointPolicyPureVector() {};
        /// Copy assignment operator
        virtual PartialJointPolicyPureVector& operator= 
            (const PartialJointPolicyPureVector& o);
        virtual PartialJointPolicyPureVector& operator= 
            (const PartialJointPolicyDiscretePure& o);
        
        bool operator< (const JointPolicy& o) const
        {
            const PartialJointPolicyPureVector& p = dynamic_cast<const PartialJointPolicyPureVector&>(o);
            return (*this) < p;
        }
        bool operator< (const PartialJointPolicyPureVector& o) const
        { return (GetIndex() < o.GetIndex()) ; }

        /// Sets the depth of the joint policy.
        /** Also sets the depth of each of the invididual policies. */
        void SetDepth(size_t d)
        {
#if 0
            if(d==MAXHORIZON)
                throw(E("PartialJointPolicyPureVector can only be used for finite horizons."));
#endif
            JointPolicy::SetDepth(d);
            JPolComponent_VectorImplementation::SetDepthForIndivPols(d);
        };

        /// Prints a description of this JointPolicyPureVector to a string.
        virtual std::string SoftPrint() const; 
        /// Prints a brief description to a string.
        virtual std::string SoftPrintBrief() const; 
        void PrintBrief() const
        { std::cout << SoftPrintBrief();}
        
        /**\brief Convert this joint policy to a JointPolicyPureVector.
         */
        virtual boost::shared_ptr<JointPolicyPureVector> ToJointPolicyPureVector() const;

    //relay the following functions to the JPolComponent_VectorImplementation:
    //
        bool operator++ ()   
        {   return( this->Increment() );     };
        bool Increment(Index agentI)
        { return this->JPolComponent_VectorImplementation::Increment(agentI); }
        bool Increment()
        { return this->JPolComponent_VectorImplementation::Increment(); }

        LIndex GetIndex() const
        {   return this->JPolComponent_VectorImplementation::GetIndex(); }
        void SetIndex(LIndex i)
        {   this->JPolComponent_VectorImplementation::SetIndex(i); }

        virtual Index GetJointActionIndex(Index i) const
        { return JPolComponent_VectorImplementation::GetJointActionIndex(i); }
        virtual Index GetJointActionIndex(LIndex i) const
        { return JPolComponent_VectorImplementation::GetJointActionIndex(i); }
        virtual Index GetActionIndex(Index agI , Index domainI) const
        { return JPolComponent_VectorImplementation::GetActionIndex(agI , domainI);}

        virtual PolicyDiscrete* GetIndividualPolicyDiscrete(Index i) const
        { return JPolComponent_VectorImplementation::GetIndividualPolicyDiscrete(i) ;}
        /// Returns a reference to the vector of pointers to individual policies
        /** This function should be used to manipulate individual policies.
         */
        std::vector<PolicyPureVector*>& GetIndividualPolicies()
        {return this->JPolComponent_VectorImplementation::GetIndividualPolicies();}
        
        virtual void ZeroInitialization()
        { JPolComponent_VectorImplementation::ZeroInitialization();}
        virtual void ZeroInitialization(Index i)
        { JPolComponent_VectorImplementation::ZeroInitialization(i);}
        virtual void RandomInitialization()
        { JPolComponent_VectorImplementation::RandomInitialization();}
        virtual void RandomInitialization(Index i)
        { JPolComponent_VectorImplementation::RandomInitialization(i);}
        virtual void SetAction(Index agentI, Index domainI, Index aI)
        { JPolComponent_VectorImplementation::SetAction(agentI, domainI, aI);}
        
        /// Returns a pointer to a copy of this class.
        virtual PartialJointPolicyPureVector* Clone() const
        { return new PartialJointPolicyPureVector(*this); }

};


#endif /* !_PARTIALJOINTPOLICYPUREVECTOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
