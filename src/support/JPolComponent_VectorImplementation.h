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
#ifndef _JPOLCOMPONENT_VECTORIMPLEMENTATION_H_
#define _JPOLCOMPONENT_VECTORIMPLEMENTATION_H_ 1

/* the include directives */
#include "Globals.h"
#include "PolicyPureVector.h"

class Interface_ProblemToPolicyDiscretePure;

/** \brief JPolComponent_VectorImplementation implements functionality
 * common to several joint policy implementations. */
class JPolComponent_VectorImplementation 
{
    private:   
       /** \brief Private function that deletes all the individual
         * policies pointed to.*/
        void ClearIndividualPolicies();
        /// vector for  temporarily storing indiv. actions 
        /**(used by GetJointActionIndex)
         */
        Index * _m_indivActionIndices;
        
        ///Aux. function used by copy constructors
        void PrivClone(const JPolComponent_VectorImplementation& o);

    // why should this class have its own pointer? That way we can never change it!
        const Interface_ProblemToPolicyDiscretePure* _m_PTPDP;
        I_PtPDpure_constPtr _m_PTPDPshared;
        PolicyDomainCategory _m_idc;

        const Interface_ProblemToPolicyDiscretePure* GetInterfacePTPDiscretePure() const
        {
            if(_m_PTPDP)
                return(_m_PTPDP);
            else
                return(_m_PTPDPshared.get());
        }
        void Construct(        
            const Interface_ProblemToPolicyDiscretePure* pu,
            PolicyGlobals::PolicyDomainCategory idc,
            size_t depth);
    
        void Construct(        
            const I_PtPDpure_constPtr &pu,
            PolicyGlobals::PolicyDomainCategory idc,
            size_t depth);
    
    protected:
        std::vector<PolicyPureVector*> _m_indivPols_PolicyPureVector;
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        JPolComponent_VectorImplementation(
            const Interface_ProblemToPolicyDiscretePure* pu,
            size_t depth=MAXHORIZON);

        JPolComponent_VectorImplementation(
            const Interface_ProblemToPolicyDiscretePure* pu,
            PolicyGlobals::PolicyDomainCategory idc,
            size_t depth=MAXHORIZON);

        JPolComponent_VectorImplementation(
            const I_PtPDpure_constPtr &pu,
            size_t depth=MAXHORIZON);

        JPolComponent_VectorImplementation(
            const I_PtPDpure_constPtr &pu,
            PolicyGlobals::PolicyDomainCategory idc,
            size_t depth=MAXHORIZON);
        /// Copy constructor.
        JPolComponent_VectorImplementation(
            const JPolComponent_VectorImplementation& a);

        /// Destructor.
        ~JPolComponent_VectorImplementation();
        /// Copy assignment operator
        JPolComponent_VectorImplementation& operator= (const
                JPolComponent_VectorImplementation& o);

        ///Performs increment. See Increment().
        bool operator++ ()   
        {    return( this->Increment() );     };
        
        /// Get the index of this joint policy.
        LIndex GetIndex() const;

        /// Returns a reference to the vector of pointers to individual policies
        /** This function should be used to manipulate individual policies.
         */
        std::vector<PolicyPureVector*>& GetIndividualPolicies()
        {return _m_indivPols_PolicyPureVector;};
        
        virtual PolicyDiscrete* GetIndividualPolicyDiscrete(Index agI) const
        { return _m_indivPols_PolicyPureVector.at(agI); }

        /// Sets the index of this joint policy.Updates the policies represented
        void SetIndex(LIndex i);

        //data manipulation (set) functions:
        ///Randomly initialize the joint policy.
        void RandomInitialization();
        ///Randomly initialize the policy for agentI.
        void RandomInitialization(Index agentI);
        /// Initialize the joint policy to the first joint policy.
        /** This is the joint policy that specifies action 0 --the
         *  first action-- for all observation histories.*/
        void ZeroInitialization();
        /// Initialize the policy of agentI to the first policy.
        /** This is the policy that specifies action 0 --the first
         *  action-- for all observation histories.*/
        void ZeroInitialization(Index agentI);


        ///Increments the individual policy of agentI.
        bool Increment(Index agentI) 
        {return( ++(*_m_indivPols_PolicyPureVector[agentI])) ;};
        ///Increments the joint policy, used to iterate over joint policies.
        bool Increment();

        void SetDepthForIndivPols(size_t d);

        ///Sets the policy for agentI to domainI->aI.
        void SetAction(Index agentI, Index domainI, Index aI)
            {_m_indivPols_PolicyPureVector[agentI]->SetAction(domainI, aI);}

        /// Prints a description of this to a string.
        std::string SoftPrint() const; 
        std::string SoftPrintBrief() const;

        
        ///Returns the jaI taken by this policy for joint domain index johI.
        Index GetJointActionIndex(Index i) const;
        ///Returns the jaI taken by this policy for joint domain index johI.
        Index GetJointActionIndex(LIndex i) const;
        ///Returns the action index for domainI for agent aI.
        Index GetActionIndex(Index aI, Index domainI) const
            {return _m_indivPols_PolicyPureVector[aI]->GetActionIndex(domainI);}
        
        


};


#endif /* !_JPOLCOMPONENT_VECTORIMPLEMENTATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
